#include "crawl_controller.hpp"

#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

namespace wl {
namespace {

constexpr double STEP_LENGTH_M = 0.03;
constexpr double BACKWARD_STEP_M = 0.02;
constexpr double TURN_ANGLE_RAD = 0.02617993877991494;  // 每轮目标偏航 1.5°。
constexpr double TWO_PI = 6.283185307179586;
// 仿真转向工作范围，保留足够的髋侧摆行程供支撑转移和落地使用。
constexpr double MAX_TURN_LATERAL_OFFSET_M = 0.065;
constexpr double MAX_TURN_HIP_ANGLE_RAD = 0.42;
constexpr double MAX_CRAWL_TILT_RAD = 0.131;  // 约 7.5°。

bool tiltWithinLimit(const SensorSample& feedback) {
    return std::cos(feedback.roll_rad) * std::cos(feedback.pitch_rad)
           >= std::cos(MAX_CRAWL_TILT_RAD);
}

void checkTurnHipAngles(const ActuatorTargets& targets) {
    for (const auto& angles : targets.leg_angles) {
        if (std::abs(angles[0]) > MAX_TURN_HIP_ANGLE_RAD) {
            throw std::domain_error("turn would exceed hip workspace");
        }
    }
}
constexpr double LIFT_HEIGHT_M = 0.045;
constexpr double SWING_DURATION_S = 2.5;
constexpr double SUPPORT_SETTLE_S = 0.5;
constexpr double PHASE_TIMEOUT_S = 5.0;
constexpr int SUPPORT_FRAMES = 16;
constexpr int LANDING_FRAMES = 8;

double swingProgress(double normalized_time) {
    // Keep the tested rise, peak hold and descent proportions.
    const double seconds = normalized_time * 5.0;
    if (seconds < 2.0) {
        return seconds / 4.0;
    }
    if (seconds < 3.0) {
        return 0.5;
    }
    return 0.5 + (seconds - 3.0) / 4.0;
}

Vec3 rotateHorizontal(const Vec3& point, double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {c * point[0] - s * point[1], s * point[0] + c * point[1], point[2]};
}

}  // namespace

CrawlController::CrawlController(webots::Supervisor& robot)
    : body_(robot.getSelf()), floor_z_m_(flatFloorHeight(robot)), gait_(SWING_DURATION_S) {
    if (!body_) {
        throw std::runtime_error("Missing Supervisor robot node");
    }
    for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
        wheels_[leg] = body_->getFromProtoDef(std::string("CRAWL_") + LEGS[leg]);
        if (!wheels_[leg]) {
            throw std::runtime_error(std::string("Missing wheel node: ") + LEGS[leg]);
        }
        wheels_[leg]->enableContactPointsTracking(static_cast<int>(robot.getBasicTimeStep()));
    }
}

void CrawlController::start(const FourLegKinematics& kinematics, const SensorSample& feedback,
                            const MotionCommand& command) {
    cycle_completed_ = false;
    if (!tiltWithinLimit(feedback)) {
        throw std::domain_error("body tilt too large for crawl");
    }
    const bool resume = paused_after_landing_ && command.forward == cycle_command_.forward
                        && command.turn == cycle_command_.turn;
    if (resume) {
        // 上一步已确认落地。从实测姿态为下一条腿重新规划。
        stop_requested_ = false;
        if (completed_steps_ == LEG_COUNT) {
            beginRecenter(kinematics, feedback);
        } else {
            beginSupportShift(kinematics, feedback);
        }
        last_targets_ = shiftTargets(kinematics, 0.0);
        paused_after_landing_ = false;
        running_ = true;
        std::cout << "Crawl resumed after landing" << std::endl;
        return;
    }
    paused_after_landing_ = false;
    cycle_command_ = command;
    gait_ = CrawlGait(SWING_DURATION_S);
    goal_ = {};
    completed_steps_ = 0;
    stop_requested_ = false;
    recentering_ = false;
    const bool was_turning = turning_;
    // 同时有直行和转向请求时，先执行一轮直行。
    turning_ = command.forward == 0.0 && command.turn != 0.0;
    step_length_m_ = turning_ ? 0.0 : (command.forward > 0.0 ? STEP_LENGTH_M : -BACKWARD_STEP_M);
    const double turn_rad = turning_ ? std::copysign(TURN_ANGLE_RAD, command.turn) : 0.0;
    forward_after_turn_ = was_turning && command.forward > 0.0;
    target_yaw_rad_ = feedback.yaw_rad + turn_rad;
    initial_wheel_offset_ = rotateHorizontal(wheelOffsetWorld(), turn_rad);

    if (turning_) {
        // 长按多轮时，每只轮子的侧向滑移可能累积。用标准站姿的轮心作基准。
        const auto nominal_centers = kinematics.forward(STAND_TARGET_ANGLES);
        const auto measured_centers = kinematics.forward(feedback.joint_angles);
        for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
            if (std::abs(measured_centers[leg][1] - nominal_centers[leg][1])
                > MAX_TURN_LATERAL_OFFSET_M) {
                throw std::domain_error("wheel positions drifted outside turn workspace");
            }
        }

        // 冻结四个落点。每个轮子绕起步时的机身原点转相同角度。
        const auto* origin = body_->getPosition();
        for (std::size_t leg = 0; leg < LEG_COUNT; ++leg) {
            const auto* center = wheels_[leg]->getPosition();
            const auto rotated = rotateHorizontal(
                {center[0] - origin[0], center[1] - origin[1], 0.0}, turn_rad);
            turn_end_world_[leg] = {origin[0] + rotated[0], origin[1] + rotated[1], center[2]};
        }
    }
    beginSupportShift(kinematics, feedback);
    last_targets_ = sampleBodyShiftTargets(kinematics, step_, start_, LEG_JOINT_LIMITS, 0.0);
    running_ = true;
    const char* direction = turning_ ? (command.turn > 0.0 ? "left" : "right")
                                     : (command.forward > 0.0 ? "forward" : "backward");
    std::cout << "Crawl started " << direction << std::endl;
}

bool CrawlController::running() const {
    return running_;
}

bool CrawlController::cycleCompleted() const {
    return cycle_completed_;
}

void CrawlController::hold(const char* reason) {
    running_ = false;
    cycle_completed_ = false;
    std::cout << "Crawl holding: " << reason << std::endl;
}

void CrawlController::clearPausedStep() {
    paused_after_landing_ = false;
}

void CrawlController::requestStop() {
    if (!running_ || stop_requested_) {
        return;
    }
    stop_requested_ = true;
    // 摆动期间先让当前轮落地，支撑平移和回中期间直接保持上一拍目标。
    if (goal_.phase == GaitPhase::ShiftSupport || recentering_) {
        hold("stop requested");
    } else {
        std::cout << "Crawl stopping after current landing" << std::endl;
    }
}

ActuatorTargets CrawlController::update(const FourLegKinematics& kinematics,
                                        const SensorSample& feedback) {
    if (!running_) {
        return last_targets_.value();
    }

    if (!tiltWithinLimit(feedback)) {
        hold("body tilt limit exceeded");
        return last_targets_.value();
    }
    try {
        const auto contacts = wheelContacts(wheels_, floor_z_m_);
        const bool all_grounded = std::all_of(contacts.begin(), contacts.end(),
                                              [](bool grounded) { return grounded; });
        const double now_s = feedback.time_s;
        const double elapsed_s = now_s - start_.time_s;
        const auto phase = goal_.phase;
        const auto leg = goal_.active_leg;
        const bool shifting = phase == GaitPhase::ShiftSupport;
        if (now_s - wait_started_s_ > PHASE_TIMEOUT_S) {
            hold("phase timed out");
            return last_targets_.value();
        }

        auto targets = last_targets_.value();
        if (shifting) {
            targets = shiftTargets(kinematics, elapsed_s);
            const bool at_target = legPosesReached(feedback.joint_angles, targets.leg_angles,
                                                   STAND_TOLERANCE_RAD);
            const bool settled = elapsed_s >= BODY_SHIFT_DURATION_S && all_grounded && at_target;
            if (recentering_) {
                support_samples_ = settled ? support_samples_ + 1 : 0;
                last_targets_ = targets;
                if (support_samples_ >= LANDING_FRAMES) {
                    std::cout << "Crawl recentered, offset error mm="
                              << 1000.0 * std::hypot(recenterErrorWorld()[0], recenterErrorWorld()[1])
                              << std::endl;
                    hold("cycle complete");
                    cycle_completed_ = true;
                }
                return targets;
            }

            const double margin_m = estimateSupportMargin(kinematics, feedback, leg);
            const bool ready = settled && margin_m > 0.0 && now_s >= wait_started_s_;
            support_samples_ = ready ? support_samples_ + 1 : 0;
        } else {
            saw_airborne_ = saw_airborne_ || (!contacts[leg] && wheelClearance(*wheels_[leg], floor_z_m_) > 0.0005);
            // 等待确认时，摆动轮必须曾离地，且四轮连续接地才算完成本步。
            const bool landed = phase == GaitPhase::WaitContact && saw_airborne_ && all_grounded;
            landing_samples_ = landed ? landing_samples_ + 1 : 0;
        }

        goal_ = gait_.update(now_s, support_samples_ >= SUPPORT_FRAMES,
                             landing_samples_ >= LANDING_FRAMES);
        if (goal_.active_leg != leg) {
            ++completed_steps_;
            std::cout << "Crawl landed " << LEGS[leg] << " (" << completed_steps_ << '/'
                      << LEG_COUNT << ')' << std::endl;
            if (stop_requested_) {
                hold("stop requested");
                paused_after_landing_ = true;
                return last_targets_.value();
            }
            if (completed_steps_ == LEG_COUNT) {
                beginRecenter(kinematics, feedback);
            } else {
                beginSupportShift(kinematics, feedback);
            }
            targets = shiftTargets(kinematics, 0.0);
        } else if (goal_.phase == GaitPhase::Swing) {
            if (shifting) {
                wait_started_s_ = now_s + SWING_DURATION_S;
                std::cout << "Crawl swing " << LEGS[leg] << std::endl;
            }
            targets = swingTargets(kinematics, swingProgress(goal_.swing_progress));
        } else if (goal_.phase == GaitPhase::WaitContact) {
            targets = swingTargets(kinematics, 1.0);
        }

        last_targets_ = targets;
        return targets;
    } catch (const std::exception& error) {
        hold(error.what());
        return last_targets_.value();
    }
}

void CrawlController::beginSupportShift(const FourLegKinematics& kinematics,
                                        const SensorSample& feedback) {
    start_ = feedback;
    step_ = makeCrawlStepPlan(kinematics, start_.joint_angles, goal_.active_leg,
                              step_length_m_, LIFT_HEIGHT_M);
    if (turning_) {
        const auto leg = goal_.active_leg;
        const auto* center = wheels_[leg]->getPosition();
        const Vec3 delta_world{turn_end_world_[leg][0] - center[0],
                               turn_end_world_[leg][1] - center[1], 0.0};
        const auto* rotation = body_->getOrientation();
        for (std::size_t axis = 0; axis < 3; ++axis) {
            step_.swing_end_m[axis] += rotation[axis] * delta_world[0]
                                       + rotation[3 + axis] * delta_world[1];
        }
    }
    auto shift = planSupportShift(kinematics, step_, start_, LEG_JOINT_LIMITS);
    // 转向和左后腿需要更大的支撑平移。前向首步的上限由起步姿态决定。
    double limit_m = 0.05;
    if (turning_ || goal_.active_leg == LEFT_HIND_LEG_INDEX) {
        limit_m = 0.07;
    } else if (step_length_m_ > 0.0 && completed_steps_ == 0) {
        limit_m = forward_after_turn_ ? 0.075 : 0.045;
    }
    const double distance_m = std::hypot(shift[0], shift[1], shift[2]);
    if (distance_m > limit_m) {
        for (double& component : shift) {
            component *= limit_m / distance_m;
        }
    }
    step_.body_shift_m = shift;
    if (turning_) {
        // 在四轮接地时检查计划中的支撑终点、抬腿峰值和落点。
        checkTurnHipAngles(shiftTargets(kinematics, BODY_SHIFT_DURATION_S));
        auto swing = goal_;
        swing.phase = GaitPhase::Swing;
        for (double progress : {0.5, 1.0}) {
            swing.swing_progress = progress;
            checkTurnHipAngles(buildCrawlTargets(swing, step_, kinematics,
                                                  LEG_JOINT_LIMITS, start_.joint_angles));
        }
    }
    wait_started_s_ = start_.time_s + BODY_SHIFT_DURATION_S;
    if (completed_steps_ == 0) {
        wait_started_s_ += 2.0 * SUPPORT_SETTLE_S;
    }
    support_samples_ = 0;
    landing_samples_ = 0;
    saw_airborne_ = false;
    std::cout << "Crawl support shift for " << LEGS[goal_.active_leg] << std::endl;
}

void CrawlController::beginRecenter(const FourLegKinematics& kinematics,
                                    const SensorSample& feedback) {
    start_ = feedback;
    step_ = makeCrawlStepPlan(kinematics, start_.joint_angles, goal_.active_leg, 0.0, 0.0);
    const auto error = recenterErrorWorld();
    const auto* rotation = body_->getOrientation();
    // 世界水平位移乘 R 的转置，得到起始机身坐标系中的目标位移。
    for (std::size_t axis = 0; axis < 3; ++axis) {
        step_.body_shift_m[axis] = rotation[axis] * error[0] + rotation[3 + axis] * error[1];
    }
    recenter_yaw_rad_ = turning_
                            ? std::remainder(target_yaw_rad_ - feedback.yaw_rad, TWO_PI)
                            : 0.0;
    recentering_ = true;
    if (turning_) {
        // 转向回中开始前检查目标。失败时继续保持已落地的姿态。
        checkTurnHipAngles(shiftTargets(kinematics, BODY_SHIFT_DURATION_S));
    }
    support_samples_ = 0;
    wait_started_s_ = start_.time_s + BODY_SHIFT_DURATION_S;
}

ActuatorTargets CrawlController::shiftTargets(const FourLegKinematics& kinematics,
                                              double elapsed_s) const {
    if (!recentering_ || !turning_) {
        return sampleBodyShiftTargets(kinematics, step_, start_, LEG_JOINT_LIMITS, elapsed_s);
    }

    // 四轮接地回中时，固定轮心并逐渐改变机身的平面朝向。
    const double progress = quinticProgress(elapsed_s, BODY_SHIFT_DURATION_S);
    const double angle_rad = -progress * recenter_yaw_rad_;
    const Vec3 up{-std::sin(start_.pitch_rad),
                  std::cos(start_.pitch_rad) * std::sin(start_.roll_rad),
                  std::cos(start_.pitch_rad) * std::cos(start_.roll_rad)};
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    auto centers = step_.reference_centers_m;
    for (auto& center : centers) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            center[axis] -= progress * step_.body_shift_m[axis];
        }
        // 在起始机身系里绕世界竖直轴做逆旋转。
        const Vec3 cross{up[1] * center[2] - up[2] * center[1],
                         up[2] * center[0] - up[0] * center[2],
                         up[0] * center[1] - up[1] * center[0]};
        const double dot = up[0] * center[0] + up[1] * center[1] + up[2] * center[2];
        for (std::size_t axis = 0; axis < 3; ++axis) {
            center[axis] = cosine * center[axis] + sine * cross[axis]
                           + (1.0 - cosine) * dot * up[axis];
        }
    }
    ActuatorTargets targets(kinematics.inverse(centers, LEG_JOINT_LIMITS, start_.joint_angles));
    targets.wheel_hold_angles = compensateWheelAngles(start_.joint_angles, start_.angles,
                                                      targets.leg_angles);
    return targets;
}

Vec3 CrawlController::wheelOffsetWorld() const {
    const auto* body_position = body_->getPosition();
    Vec3 offset{-body_position[0], -body_position[1], 0.0};
    for (const auto* wheel : wheels_) {
        const auto* center = wheel->getPosition();
        for (std::size_t axis = 0; axis < 2; ++axis) {
            offset[axis] += center[axis] / LEG_COUNT;
        }
    }
    return offset;
}

Vec3 CrawlController::recenterErrorWorld() const {
    const auto offset = wheelOffsetWorld();
    return {offset[0] - initial_wheel_offset_[0], offset[1] - initial_wheel_offset_[1], 0.0};
}

ActuatorTargets CrawlController::swingTargets(const FourLegKinematics& kinematics,
                                              double progress) const {
    auto swing = goal_;
    swing.swing_progress = progress;
    auto targets = buildCrawlTargets(swing, step_, kinematics, LEG_JOINT_LIMITS,
                                     start_.joint_angles);
    targets.wheel_hold_angles = compensateWheelAngles(start_.joint_angles, start_.angles,
                                                      targets.leg_angles);
    return targets;
}

}  // namespace wl
