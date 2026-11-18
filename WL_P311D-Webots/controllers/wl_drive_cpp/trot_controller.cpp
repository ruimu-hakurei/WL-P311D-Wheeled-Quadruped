// Transfer 转移重心，Swing 摆腿并推进，Landing 确认落地，Recenter 恢复站姿。
#include "trot_controller.hpp"

#include "body_control.hpp"
#include "config.hpp"
#include "estimation.hpp"
#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

namespace wl {
namespace {

constexpr double TROT_RECENTER_S = 1.50;
constexpr double SHIFT_S = 0.16;
constexpr double SWING_S = 0.42;
// 这是把前进输入换算为落点步幅的标定系数，单位 s。
constexpr double STRIDE_SCALE_S = 1.04;
// 平地直行的加长步幅。原地转和弧线沿用各自的转向参数。
constexpr double STRAIGHT_STRIDE_MULTIPLIER = 1.16;
constexpr double CRUISE_STRIDE_MULTIPLIER = 1.27;
constexpr double STRIDE_RAMP_TRANSFERS = 4.0;
constexpr std::size_t MOVING_TRANSFERS_BEFORE_RECENTER = 24;
constexpr std::size_t TURN_TRANSFERS_BEFORE_RECENTER = 6;
constexpr std::size_t MIN_STEPS_BEFORE_WORKSPACE_CHECK = 8;
constexpr double MAX_FORWARD_WHEEL_OFFSET_M = 0.12;
constexpr double MAX_LATERAL_WHEEL_OFFSET_M = 0.075;

// 摆动期间规划的机身前移和偏航份额，以及落点向名义站姿靠拢的权重。
constexpr double SWING_ADVANCE_FRACTION = 0.55;
// 摆动阶段机身主动转的份额。弧线中让支撑腿参与更多转向，减小换向时的回摆。
constexpr double ARC_BODY_YAW_FRACTION = 0.53;
constexpr double PIVOT_BODY_YAW_FRACTION = 0.40;
constexpr double NOMINAL_FOOT_BLEND = 0.40;
constexpr double LIFT_M = 0.036;
constexpr double TURN_LIFT_M = 0.039;

// 航向误差和横向位置误差分别反馈到落点，修正量都有限幅。
constexpr double HEADING_GAIN = 0.38;
constexpr double MAX_STEP_YAW_RAD = 0.035;
constexpr double LATERAL_GAIN = 0.18;
constexpr double MAX_LATERAL_STEP_M = 0.025;
constexpr double TURN_CENTER_GAIN = 0.45;
constexpr double MAX_TURN_CORRECTION_M = 0.045;
constexpr double FULL_TURN_RAD = 6.283185307179586;
constexpr double MAX_SHIFT_M = 0.09;
constexpr double MAX_TILT_RAD = 0.15;
constexpr double MIN_CLEARANCE_M = 0.001;
constexpr double SUPPORT_FEEDBACK_RAMP_S = 0.14;
constexpr double SUPPORT_FEEDBACK_GAIN = 0.50;
constexpr double MAX_SUPPORT_CORRECTION_M = 0.018;
constexpr double LANDING_TIMEOUT_S = 0.50;
constexpr int CONTACT_CONFIRM_FRAMES = 4;
// 直行支撑转移和落地用连续两次接触采样。转弯和回中仍用四次。
constexpr int STRAIGHT_CONTACT_FRAMES = 2;

// 同一行的两条腿同时摆动。顺序为 LF、RF、LH、RH。
constexpr std::array<std::array<bool, LEG_COUNT>, 2> SWING_LEGS{{
    {true, false, false, true},
    {false, true, true, false}
}};

Vec3 toBody(const double* rotation, const Vec3& world) {
    // orientation 按行存储机身到世界的矩阵 R。点积取 R 的列，相当于乘 R 转置。
    // 这里处理的是位移向量，只转换旋转。
    return {
        rotation[0] * world[0] + rotation[3] * world[1] + rotation[6] * world[2],
        rotation[1] * world[0] + rotation[4] * world[1] + rotation[7] * world[2],
        rotation[2] * world[0] + rotation[5] * world[1] + rotation[8] * world[2],
    };
}

}  // namespace

TrotController::TrotController(webots::Supervisor& robot, const FourLegKinematics& kinematics,
                               bool log_contacts)
    : kinematics_(kinematics), nominal_centers_(kinematics.forward(STAND_TARGET_ANGLES)),
      log_contacts_(log_contacts), body_(robot.getSelf()), floor_z_m_(flatFloorHeight(robot)) {
    if (!body_) {
        throw std::runtime_error("Missing robot supervisor node");
    }
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        // auto* 让编译器推断指针类型。DEF 名称与机器人 PROTO 中的轮节点对应。
        auto* wheel = body_->getFromProtoDef(std::string("CRAWL_") + LEGS[leg]);
        if (!wheel) {
            throw std::runtime_error(std::string("Missing wheel node: ") + LEGS[leg]);
        }
        wheel->enableContactPointsTracking(static_cast<int>(robot.getBasicTimeStep()));
        wheels_[leg] = wheel;
    }
}

bool TrotController::swings(std::size_t leg) const {
    return SWING_LEGS[diagonal_][leg];
}

void TrotController::beginRecenter(const SensorSample& feedback) {
    phase_ = Phase::Recenter;
    phase_start_ = feedback;
    recenter_.emplace(feedback.joint_angles, STAND_TARGET_ANGLES, feedback.time_s, TROT_RECENTER_S);
    grounded_frames_ = 0;
    std::cout << "TROT recenter " << feedback.time_s << std::endl;
}

ActuatorTargets TrotController::sampleRecenter(const SensorSample& feedback) const {
    ActuatorTargets targets(recenter_->sample(feedback.time_s));
    targets.wheel_hold_angles = compensateWheelAngles(phase_start_.joint_angles,
                                                       phase_start_.angles, targets.leg_angles);
    return targets;
}

void TrotController::beginTransfer(const SensorSample& feedback, double forward_m_s,
                                   double yaw_step_rad) {
    phase_ = Phase::Transfer;
    phase_start_ = feedback;
    references_ = kinematics_.forward(feedback.joint_angles);
    // 从站姿起步时逐步放大步幅，等支撑布局形成后再进入巡航。
    const double ramp = std::min(1.0, static_cast<double>(moving_transfers_) / STRIDE_RAMP_TRANSFERS);
    const double straight_stride = STRAIGHT_STRIDE_MULTIPLIER
                                   + ramp * (CRUISE_STRIDE_MULTIPLIER - STRAIGHT_STRIDE_MULTIPLIER);
    const double stride_multiplier = yaw_step_rad == 0.0 ? straight_stride : 1.0;
    step_m_ = forward_m_s * STRIDE_SCALE_S * stride_multiplier;
    step_yaw_rad_ = yaw_step_rad;
    turning_ = yaw_step_rad != 0.0;
    step_lateral_m_ = 0.0;
    turn_center_correction_ = {};
    if (turning_ && forward_m_s == 0.0) {
        if (!turn_origin_m_) {
            turn_origin_m_ = {feedback.reference_center_x_m, feedback.reference_center_y_m};
        }
        // 单独转向时，落点同时拉回开始转弯时的中心。前进转弯时允许机身平移。
        const double dx = (*turn_origin_m_)[0] - feedback.reference_center_x_m;
        const double dy = (*turn_origin_m_)[1] - feedback.reference_center_y_m;
        const double scale = std::max(1.0, TURN_CENTER_GAIN * std::hypot(dx, dy)
                                          / MAX_TURN_CORRECTION_M);
        turn_center_correction_ = toBody(body_->getOrientation(),
                                         {TURN_CENTER_GAIN * dx / scale,
                                          TURN_CENTER_GAIN * dy / scale, 0.0});
    } else {
        turn_origin_m_.reset();
    }
    if (forward_m_s > 0.0 && !turning_) {
        if (!forward_heading_rad_) {
            forward_heading_rad_ = feedback.yaw_rad;
            forward_origin_m_ = {feedback.reference_center_x_m, feedback.reference_center_y_m};
        }
        const double error = std::remainder(*forward_heading_rad_ - feedback.yaw_rad,
                                            FULL_TURN_RAD);
        step_yaw_rad_ = std::clamp(HEADING_GAIN * error, -MAX_STEP_YAW_RAD, MAX_STEP_YAW_RAD);
        // 航向对准时也可能整机横移。用起步时的直线计算有符号横向距离。
        const double dx = feedback.reference_center_x_m - forward_origin_m_[0];
        const double dy = feedback.reference_center_y_m - forward_origin_m_[1];
        const double lateral_error_m = -std::sin(*forward_heading_rad_) * dx
                                       + std::cos(*forward_heading_rad_) * dy;
        step_lateral_m_ = std::clamp(-LATERAL_GAIN * lateral_error_m,
                                    -MAX_LATERAL_STEP_M, MAX_LATERAL_STEP_M);
    }

    // 对角支撑在地面上接近一条线。先把估算质心沿水平方向移向该连线。
    std::array<Vec3, 2> support{};
    std::size_t index = 0;
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        if (!swings(leg)) {
            const auto* point = wheels_[leg]->getPosition();
            support[index++] = {point[0], point[1], point[2]};
        }
    }
    const Vec3 com_body = estimateCenterOfMass(kinematics_, feedback.joint_angles,
                                               feedback.angles);
    const auto* position = body_->getPosition();
    const auto* rotation = body_->getOrientation();
    const double com_x = position[0] + rotation[0] * com_body[0] + rotation[1] * com_body[1]
                         + rotation[2] * com_body[2];
    const double com_y = position[1] + rotation[3] * com_body[0] + rotation[4] * com_body[1]
                         + rotation[5] * com_body[2];
    const double dx = support[1][0] - support[0][0];
    const double dy = support[1][1] - support[0][1];
    if (std::hypot(dx, dy) < 0.01) {
        throw std::runtime_error("degenerate diagonal support line");
    }
    // 投影系数 0 对应第一只支撑轮，1 对应第二只。限在中段给两端留余量。
    const double along = std::clamp(
        ((com_x - support[0][0]) * dx + (com_y - support[0][1]) * dy) / (dx * dx + dy * dy),
        0.30, 0.70);
    const Vec3 shift{support[0][0] + along * dx - com_x, support[0][1] + along * dy - com_y, 0.0};
    const double scale = std::max(1.0, std::hypot(shift[0], shift[1]) / MAX_SHIFT_M);
    body_shift_ = toBody(rotation, {shift[0] / scale, shift[1] / scale, 0.0});
    grounded_frames_ = 0;
    saw_airborne_.fill(false);
    std::cout << "TROT transfer " << feedback.time_s << ' ' << diagonal_ << std::endl;
}

void TrotController::beginSwing(const SensorSample& feedback) {
    phase_ = Phase::Swing;
    phase_start_ = feedback;
    references_ = kinematics_.forward(feedback.joint_angles);
    grounded_frames_ = 0;

    // 两个支撑轮构成一条线。只取前进请求在该方向上的投影，减少横向失衡。
    Vec3 support_direction{};
    std::size_t support_index = 0;
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        if (!swings(leg)) {
            const auto* point = wheels_[leg]->getPosition();
            const double sign = support_index++ == 0 ? -1.0 : 1.0;
            support_direction[0] += sign * point[0];
            support_direction[1] += sign * point[1];
        }
    }
    const double support_length_m = std::hypot(support_direction[0], support_direction[1]);
    if (support_length_m < 0.01) {
        throw std::runtime_error("degenerate diagonal support line at swing start");
    }
    const double support_x = support_direction[0] / support_length_m;
    const double support_y = support_direction[1] / support_length_m;
    const auto* rotation = body_->getOrientation();
    const double along_support_m = SWING_ADVANCE_FRACTION * step_m_
                                   * (rotation[0] * support_x + rotation[3] * support_y);
    swing_body_advance_ = toBody(rotation, {along_support_m * support_x,
                                             along_support_m * support_y, 0.0});
    std::cout << "TROT swing " << feedback.time_s << ' ' << diagonal_ << std::endl;
}

Vec3 TrotController::swingEnd(std::size_t leg) const {
    auto target = references_[leg];
    const double cos_yaw = std::cos(step_yaw_rad_);
    const double sin_yaw = std::sin(step_yaw_rad_);
    target[0] = cos_yaw * references_[leg][0] - sin_yaw * references_[leg][1] + step_m_;
    target[1] = sin_yaw * references_[leg][0] + cos_yaw * references_[leg][1];

    // Pupper 的落点以默认站姿为基准，并沿速度方向加上约半个支撑行程。
    // 这里平滑混合名义落点，减弱前一步位置误差的累积。高度仍沿用起摆高度。
    if (step_m_ > 0.0 || turning_) {
        const auto& nominal = nominal_centers_[leg];
        const double nominal_x = cos_yaw * nominal[0] - sin_yaw * nominal[1] + 0.5 * step_m_;
        const double nominal_y = sin_yaw * nominal[0] + cos_yaw * nominal[1];
        target[0] += NOMINAL_FOOT_BLEND * (nominal_x + swing_body_advance_[0] - target[0]);
        target[1] += NOMINAL_FOOT_BLEND * (nominal_y + swing_body_advance_[1] - target[1]);
        target[1] += step_lateral_m_;
        target[0] += turn_center_correction_[0];
        target[1] += turn_center_correction_[1];
    }
    return target;
}

WheelCenters TrotController::sampleTransferCenters(double elapsed_s) const {
    auto centers = references_;
    const double blend = quinticProgress(elapsed_s, SHIFT_S);
    // auto& 修改数组中的轮心。机身向前挪，同一固定轮心在机身坐标中向后移。
    for (auto& center : centers) {
        for (std::size_t axis = 0; axis < center.size(); ++axis) {
            center[axis] -= blend * body_shift_[axis];
        }
    }
    return centers;
}

WheelCenters TrotController::sampleSwingCenters(const SensorSample& feedback, double elapsed_s) const {
    auto centers = references_;
    const double progress = std::clamp(elapsed_s / SWING_S, 0.0, 1.0);
    // 轮心轨迹用起摆时的机身坐标表示，支撑腿的高度闭环补偿横滚和俯仰。
    // 小角度下，横滚角乘侧向距离、俯仰角乘前后距离，得到高度误差的一阶近似。
    const double roll = feedback.roll_rad - phase_start_.roll_rad;
    const double pitch = feedback.pitch_rad - phase_start_.pitch_rad;
    const double blend = quinticProgress(elapsed_s, SUPPORT_FEEDBACK_RAMP_S);
    const double advance_blend = quinticProgress(elapsed_s, SWING_S);
    // 转向时让支撑腿也带动机身。把所有轮心变换到前移和转动后的机身系。
    const double yaw_fraction = step_m_ > 0.0 ? ARC_BODY_YAW_FRACTION : PIVOT_BODY_YAW_FRACTION;
    const double body_yaw = turning_ ? advance_blend * yaw_fraction * step_yaw_rad_ : 0.0;
    const double cos_body_yaw = std::cos(body_yaw);
    const double sin_body_yaw = std::sin(body_yaw);
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        if (swings(leg)) {
            const double lift_m = turning_ ? TURN_LIFT_M : LIFT_M;
            centers[leg] = swingPosition(references_[leg], swingEnd(leg), progress, lift_m);
        } else {
            const double height = roll * (centers[leg][1] - body_shift_[1])
                                  - pitch * (centers[leg][0] - body_shift_[0]);
            centers[leg][2] += blend * std::clamp(SUPPORT_FEEDBACK_GAIN * height,
                -MAX_SUPPORT_CORRECTION_M, MAX_SUPPORT_CORRECTION_M);
        }
        // 轮心轨迹在起摆时的机身坐标中规划，转成前移后的机身坐标再做逆解。
        // 支撑轮世界位置保持原目标，腿相对机身向后运动，推动机身前移。
        for (std::size_t axis = 0; axis < swing_body_advance_.size(); ++axis) {
            centers[leg][axis] -= advance_blend * swing_body_advance_[axis];
        }
        const double x = centers[leg][0];
        const double y = centers[leg][1];
        centers[leg][0] = cos_body_yaw * x + sin_body_yaw * y;
        centers[leg][1] = -sin_body_yaw * x + cos_body_yaw * y;
    }
    return centers;
}

ActuatorTargets TrotController::sampleTargets(const SensorSample& feedback) const {
    const double elapsed_s = feedback.time_s - phase_start_.time_s;
    const auto centers = phase_ == Phase::Transfer
                             ? sampleTransferCenters(elapsed_s)
                             : sampleSwingCenters(feedback, elapsed_s);
    // 用起相位的实测关节角选相近逆解，再补偿腿部转动带来的轮轴角变化。
    ActuatorTargets targets(kinematics_.inverse(centers, LEG_JOINT_LIMITS,
                                               phase_start_.joint_angles));
    targets.wheel_hold_angles = compensateWheelAngles(phase_start_.joint_angles,
                                                       phase_start_.angles, targets.leg_angles);
    return targets;
}

bool TrotController::needsWorkspaceRecenter(const SensorSample& feedback) const {
    // 取实测轮心与站姿轮心的差，判断是否快用完水平摆腿余量。
    const auto centers = kinematics_.forward(feedback.joint_angles);
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        if (std::abs(centers[leg][0] - nominal_centers_[leg][0]) > MAX_FORWARD_WHEEL_OFFSET_M
            || std::abs(centers[leg][1] - nominal_centers_[leg][1]) > MAX_LATERAL_WHEEL_OFFSET_M) {
            return true;
        }
    }
    return false;
}

void TrotController::fail(const std::string& reason) {
    phase_ = Phase::Failed;
    if (first_fault_.empty()) {
        first_fault_ = reason;
    }
}

void TrotController::onFeedbackLost() {
    ready_to_exit_ = false;
    if (phase_ != Phase::Failed) {
        fail("feedback unavailable");
        std::cout << "TROT fault feedback unavailable" << std::endl;
    }
}

bool TrotController::beginNextStep(const SensorSample& feedback, double forward_m_s,
                                   double yaw_step_rad, bool grounded) {
    if (forward_m_s <= 0.0 || yaw_step_rad != 0.0) {
        forward_heading_rad_.reset();
    }
    if (forward_m_s < 0.0 || !grounded) {
        if (forward_m_s < 0.0) {
            turn_origin_m_.reset();
        }
        return false;
    }
    Motion motion = Motion::Forward;
    if (yaw_step_rad > 0.0) {
        motion = forward_m_s > 0.0 ? Motion::ArcLeft : Motion::PivotLeft;
    } else if (yaw_step_rad < 0.0) {
        motion = forward_m_s > 0.0 ? Motion::ArcRight : Motion::PivotRight;
    }
    // 达到次数上限或直行轮心偏离太多时，先回到站姿附近，让下一步有足够行程。
    const auto recenter_after = motion == Motion::Forward ? MOVING_TRANSFERS_BEFORE_RECENTER
                                                          : TURN_TRANSFERS_BEFORE_RECENTER;
    const bool workspace_low = motion == Motion::Forward
                               && moving_transfers_ >= MIN_STEPS_BEFORE_WORKSPACE_CHECK
                               && needsWorkspaceRecenter(feedback);
    if (workspace_low && moving_transfers_ < recenter_after) {
        std::cout << "TROT workspace recenter " << feedback.time_s
                  << ' ' << moving_transfers_ << '\n';
    }
    // 两种弧线的支撑节奏相同，左右换向可在一对腿落地后切换落点。
    // 达到换步上限时仍做回中。直行和原地转等走法之间继续先回中。
    // previous_motion_ 是 optional。直接与枚举值比较时，空值不会匹配任何方向。
    const bool reversing_arc =
        (previous_motion_ == Motion::ArcLeft && motion == Motion::ArcRight)
        || (previous_motion_ == Motion::ArcRight && motion == Motion::ArcLeft);
    const bool changes_motion = previous_motion_ && *previous_motion_ != motion;
    const bool requires_recenter = changes_motion && !reversing_arc;
    if (workspace_low || moving_transfers_ >= recenter_after || requires_recenter) {
        beginRecenter(feedback);
    } else {
        previous_motion_ = motion;
        beginTransfer(feedback, forward_m_s, yaw_step_rad);
    }
    return true;
}

void TrotController::updateRecenter(const SensorSample& feedback, double elapsed_s,
                                     bool grounded, bool exit_requested) {
    last_targets_ = sampleRecenter(feedback);
    const bool reached = elapsed_s >= TROT_RECENTER_S
                         && legPosesReached(feedback.joint_angles, STAND_TARGET_ANGLES,
                                            STAND_TOLERANCE_RAD);
    // 关节到位和四轮接触必须同时成立，任一条件断开就重新计数。
    grounded_frames_ = reached && grounded ? grounded_frames_ + 1 : 0;
    if (grounded_frames_ >= CONTACT_CONFIRM_FRAMES) {
        moving_transfers_ = 0;
        previous_motion_.reset();
        recenter_.reset();
        phase_ = Phase::Idle;
        ready_to_exit_ = exit_requested;
        std::cout << "TROT recenter done " << feedback.time_s << std::endl;
    } else if (elapsed_s > TROT_RECENTER_S + 1.5) {
        throw std::runtime_error("recenter did not settle");
    }
}

bool TrotController::updateSwingAndLanding(const SensorSample& feedback,
                                          const Contacts& contacts, double tilt_rad,
                                          double elapsed_s, bool grounded, int contact_frames) {
    std::array<double, LEG_COUNT> gaps_m{};
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        gaps_m[leg] = wheelClearance(*wheels_[leg], floor_z_m_);
        // 一旦确认该轮离地，就保留这个事实直到本步结束。
        saw_airborne_[leg] |= !contacts[leg] && gaps_m[leg] > MIN_CLEARANCE_M;
    }
    if (log_contacts_) {
        std::cout << "TROT_SAMPLE " << feedback.time_s << ' ' << diagonal_ << ' ';
        for (const bool contact : contacts) {
            std::cout << (contact ? '1' : '0');
        }
        std::cout << ' ' << tilt_rad;
        for (const double gap_m : gaps_m) {
            std::cout << ' ' << gap_m;
        }
        std::cout << '\n';
    }

    if (elapsed_s < SWING_S) {
        return false;
    }

    // 轨迹计时到期后，继续发送末端目标，等待实测接触连续稳定。
    phase_ = Phase::Landing;
    grounded_frames_ = grounded ? grounded_frames_ + 1 : 0;
    if (grounded_frames_ < contact_frames) {
        if (elapsed_s > SWING_S + LANDING_TIMEOUT_S) {
            throw std::runtime_error("landing contact timeout");
        }
        return false;
    }

    // 比较本次实测离地的四轮掩码与计划。array 的比较会逐一检查对应元素。
    if (saw_airborne_ != SWING_LEGS[diagonal_]) {
        throw std::runtime_error("wrong wheels lifted or a swing wheel stayed down");
    }
    ++count_;
    if (step_m_ > 0.0 || turning_) {
        ++moving_transfers_;
    }
    std::cout << "TROT landed " << feedback.time_s << ' ' << diagonal_ << ' '
              << count_ << ' ' << feedback.reference_center_x_m << ' '
              << feedback.reference_center_y_m << ' ' << feedback.yaw_rad << std::endl;
    diagonal_ = 1 - diagonal_;
    phase_ = Phase::Idle;
    return true;
}

ActuatorTargets TrotController::update(const SensorSample& feedback, double forward_m_s,
                                       double yaw_step_rad, bool exit_requested) {
    if (!last_targets_) {
        last_targets_ = ActuatorTargets(feedback.joint_angles);
        last_targets_->wheel_hold_angles = feedback.angles;
    }
    if (phase_ == Phase::Failed && !exit_requested) {
        return *last_targets_;
    }
    const double tilt = std::acos(std::clamp(std::cos(feedback.roll_rad)
                                           * std::cos(feedback.pitch_rad), -1.0, 1.0));
    if (tilt > MAX_TILT_RAD) {
        fail("tilt limit exceeded");
        if (!exit_requested) {
            std::cout << "TROT fault tilt " << feedback.time_s << std::endl;
        }
        return *last_targets_;
    }
    try {
        const auto contacts = wheelContacts(wheels_, floor_z_m_);
        // all_of 对每一轮调用 lambda，只有四个返回值都为 true 才算站稳。
        const bool grounded = std::all_of(contacts.begin(), contacts.end(),
                                         [](bool contact) { return contact; });
        if (exit_requested && grounded && (phase_ == Phase::Idle || phase_ == Phase::Failed)) {
            // R 在摆动中只登记退出请求。等落地后从当前实测姿态生成回中轨迹。
            beginRecenter(feedback);
        }
        if (phase_ == Phase::Failed) {
            return *last_targets_;
        }
        if (phase_ == Phase::Idle && !beginNextStep(feedback, forward_m_s, yaw_step_rad, grounded)) {
            return *last_targets_;
        }
        const double elapsed_s = feedback.time_s - phase_start_.time_s;
        if (phase_ == Phase::Recenter) {
            updateRecenter(feedback, elapsed_s, grounded, exit_requested);
            return *last_targets_;
        }
        const int contact_frames = turning_ ? CONTACT_CONFIRM_FRAMES : STRAIGHT_CONTACT_FRAMES;
        if (phase_ == Phase::Transfer) {
            grounded_frames_ = elapsed_s >= SHIFT_S && grounded ? grounded_frames_ + 1 : 0;
            if (grounded_frames_ >= contact_frames) {
                beginSwing(feedback);
            } else if (elapsed_s > SHIFT_S + LANDING_TIMEOUT_S) {
                throw std::runtime_error("support transfer did not settle");
            }
        } else if (updateSwingAndLanding(feedback, contacts, tilt, elapsed_s, grounded, contact_frames)) {
            // 本对腿已经落地。下一拍用新的实测姿态规划另一对。
            return *last_targets_;
        }
        last_targets_ = sampleTargets(feedback);
    } catch (const std::exception& error) {
        fail(error.what());
        std::cout << "TROT fault " << feedback.time_s << ' ' << error.what() << std::endl;
    }
    return *last_targets_;
}

}  // namespace wl
