// 【设备层实现】构造时查找/初始化，逐拍读取，最后检查并下发执行目标。
// 这里的速度差分和误差统计是基础处理，目前没有低通滤波或传感器融合。
#include "robot_io.hpp"
#include "rolling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wl {
namespace {

// 同一个空指针检查用于不同 SDK 设备类型；模板不承担控制算法。
template <class Device> Device* requireDevice(Device* device, const std::string& device_name) {
    if (device == nullptr) {
        throw std::runtime_error("Required device not found: " + device_name);
    }
    return device;
}

Vec3 copySensorVector(const double* values) {
    if (values == nullptr) {
        throw std::runtime_error("Sensor returned null vector");
    }
    return {values[0], values[1], values[2]};
}

bool isFiniteVector(const Vec3& vector) {
    for (double component : vector) {
        if (!std::isfinite(component)) {
            return false;
        }
    }
    return true;
}

std::string jointName(std::size_t leg_index, std::size_t joint_index) {
    return std::string(LEGS[leg_index]) + "_" + LEG_JOINTS[joint_index];
}

}  // namespace

RobotIO::RobotIO(webots::Robot& robot, int time_step_ms) {
    if (time_step_ms <= 0) {
        throw std::invalid_argument("Sampling period must be positive");
    }
    // 先确保设备全部存在，再统一初始化；构造函数结束时对象才可投入控制循环。
    findDevices(robot);
    initializeDevices(time_step_ms);
}

void RobotIO::findDevices(webots::Robot& robot) {
    // 先取无穷，再对四个轮子的最大速度取最小值，得到全车共用的保守速度上限。
    max_wheel_speed_ = std::numeric_limits<double>::infinity();

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            const std::string motor_name = jointName(leg_index, joint_index);
            const std::string encoder_name = motor_name + "_sensor";
            leg_motors_[leg_index][joint_index] =
                requireDevice(robot.getMotor(motor_name), motor_name);
            leg_encoders_[leg_index][joint_index] =
                requireDevice(robot.getPositionSensor(encoder_name), encoder_name);
        }

        const std::string wheel_name = std::string(LEGS[leg_index]) + "_WHL";
        const std::string encoder_name = wheel_name + "_sensor";
        wheels_[leg_index] = requireDevice(robot.getMotor(wheel_name), wheel_name);
        wheel_encoders_[leg_index] =
            requireDevice(robot.getPositionSensor(encoder_name), encoder_name);

        const double speed_limit = wheels_[leg_index]->getMaxVelocity();
        if (!std::isfinite(speed_limit) || speed_limit <= 0.0) {
            throw std::runtime_error("Invalid wheel limit: " + wheel_name);
        }
        max_wheel_speed_ = std::min(max_wheel_speed_, speed_limit);
    }

    const std::string imu_name = "ground_truth/state inertial";
    const std::string gps_name = "ground_truth/state gps";
    const std::string gyro_name = "ground_truth/state gyro";
    imu_ = requireDevice(robot.getInertialUnit(imu_name), imu_name);
    gps_ = requireDevice(robot.getGPS(gps_name), gps_name);
    gyro_ = requireDevice(robot.getGyro(gyro_name), gyro_name);
}

void RobotIO::initializeDevices(int time_step_ms) {
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            auto* joint_motor = leg_motors_[leg_index][joint_index];
            // 有限 position 选择位置目标；此时 velocity=1 是运动速度限制（rad/s）。
            joint_motor->setPosition(0.0);
            joint_motor->setVelocity(1.0);
            leg_encoders_[leg_index][joint_index]->enable(time_step_ms);
        }

        auto* wheel_motor = wheels_[leg_index];
        // 无穷 position 切到连续旋转的速度模式，此后 velocity 才是目标轮速。
        // 启动时采用速度模式；后续 applyTargets 可根据保持角度切换为位置控制。
        wheel_motor->setPosition(std::numeric_limits<double>::infinity());
        wheel_motor->setVelocity(0.0);
        wheel_motor->setAcceleration(WHEEL_ACCELERATION);
        wheel_encoders_[leg_index]->enable(time_step_ms);
    }

    imu_->enable(time_step_ms);
    gps_->enable(time_step_ms);
    gyro_->enable(time_step_ms);
}

std::optional<SensorSample> RobotIO::read(double now_s) {
    SensorSample feedback = readMeasurements(now_s);
    if (!hasValidFeedback(feedback)) {
        // 无效帧打断差分历史：恢复后的第一帧不跨越这段未知区间求速度。
        last_time_s_.reset();
        return std::nullopt;
    }

    estimateSpeeds(feedback);

    // 差分完成后，再保存本帧，供下一次有效采样使用。
    last_time_s_ = now_s;
    previous_wheel_angles_ = feedback.angles;
    previous_joint_angles_ = feedback.joint_angles;

    updateDisplacements(feedback);
    updateJointErrors(feedback);
    return feedback;
}

SensorSample RobotIO::readMeasurements(double now_s) const {
    SensorSample feedback;
    feedback.time_s = now_s;

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        feedback.angles[leg_index] = wheel_encoders_[leg_index]->getValue();
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            feedback.joint_angles[leg_index][joint_index] =
                leg_encoders_[leg_index][joint_index]->getValue();
            feedback.joint_targets[leg_index][joint_index] =
                leg_motors_[leg_index][joint_index]->getTargetPosition();
        }
    }

    // 直接读取 Webots 的姿态、GPS 和陀螺仪；不是通过 gyro 积分得到 RPY。
    // copySensorVector 拷贝 SDK 返回的三元素数据，不把临时数据指针放进反馈。
    const Vec3 attitude = copySensorVector(imu_->getRollPitchYaw());
    feedback.roll_rad = attitude[0];
    feedback.pitch_rad = attitude[1];
    feedback.yaw_rad = attitude[2];
    feedback.position = copySensorVector(gps_->getValues());
    feedback.world_velocity = copySensorVector(gps_->getSpeedVector());
    feedback.angular_velocity = copySensorVector(gyro_->getValues());
    return feedback;
}

bool RobotIO::hasValidFeedback(const SensorSample& feedback) {
    const Vec3 attitude{feedback.roll_rad, feedback.pitch_rad, feedback.yaw_rad};
    if (!std::isfinite(feedback.time_s) || !isFiniteVector(attitude)
        || !isFiniteVector(feedback.position) || !isFiniteVector(feedback.world_velocity)
        || !isFiniteVector(feedback.angular_velocity)) {
        return false;
    }

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        if (!std::isfinite(feedback.angles[leg_index])
            || !isFiniteVector(feedback.joint_angles[leg_index])
            || !isFiniteVector(feedback.joint_targets[leg_index])) {
            return false;
        }
    }
    return true;
}

void RobotIO::estimateSpeeds(SensorSample& feedback) const {
    if (!last_time_s_.has_value() || feedback.time_s <= last_time_s_.value()) {
        return;
    }

    // 差分速度使用本次与上次的真实时间差；首帧没有历史，speeds_valid 保持 false。
    // 这是数值差分，未来加入编码器噪声时需要考虑噪声被差分放大的问题。
    const double elapsed_s = feedback.time_s - last_time_s_.value();
    feedback.speeds_valid = true;
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        feedback.speeds[leg_index] =
            (feedback.angles[leg_index] - previous_wheel_angles_[leg_index]) / elapsed_s;
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            const double angle_change = feedback.joint_angles[leg_index][joint_index]
                                        - previous_joint_angles_[leg_index][joint_index];
            feedback.joint_speeds[leg_index][joint_index] = angle_change / elapsed_s;
        }
    }
}

void RobotIO::updateDisplacements(SensorSample& feedback) {
    if (!start_world_position_.has_value()) {
        start_world_position_ = feedback.position;
    }
    const auto& start = start_world_position_.value();
    feedback.horizontal_displacement_m =
        std::hypot(feedback.position[0] - start[0], feedback.position[1] - start[1]);

    // GPS 点沿机身 X 轴偏移到固定参考点。该点不表示质心或实际转动中心。
    feedback.reference_center_x_m =
        feedback.position[0]
        + REFERENCE_CENTER_X_M * std::cos(feedback.pitch_rad) * std::cos(feedback.yaw_rad);
    feedback.reference_center_y_m =
        feedback.position[1]
        + REFERENCE_CENTER_X_M * std::cos(feedback.pitch_rad) * std::sin(feedback.yaw_rad);

    if (!reference_point_origin_.has_value()) {
        reference_point_origin_ =
            std::array<double, 2>{feedback.reference_center_x_m, feedback.reference_center_y_m};
    }
    const auto& origin = reference_point_origin_.value();
    feedback.reference_center_displacement_m = std::hypot(
        feedback.reference_center_x_m - origin[0], feedback.reference_center_y_m - origin[1]);
}

void RobotIO::updateJointErrors(SensorSample& feedback) const {
    feedback.worst_joint = jointName(0, 0);
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            const double signed_error = feedback.joint_targets[leg_index][joint_index]
                                        - feedback.joint_angles[leg_index][joint_index];
            feedback.joint_errors[leg_index][joint_index] = signed_error;

            const double absolute_error = std::abs(signed_error);
            if (absolute_error > feedback.max_joint_error_rad) {
                feedback.max_joint_error_rad = absolute_error;
                feedback.worst_joint = jointName(leg_index, joint_index);
            }
        }
    }
}

void RobotIO::drive(double forward, double turn) {
    const WheelCommand command = differentialWheelCommand(forward, turn, max_wheel_speed_);

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        const bool left_side =
            leg_index == LEFT_FRONT_LEG_INDEX || leg_index == LEFT_HIND_LEG_INDEX;

        const double wheel_speed = left_side ? command.left : command.right;

        // 先切回连续旋转的速度模式。
        wheels_[leg_index]->setPosition(std::numeric_limits<double>::infinity());

        // 再发送目标轮速。
        wheels_[leg_index]->setVelocity(wheel_speed);
    }

    applied_command_ = command;
    // 轮子已切回速度模式，保持目标记录也随之清空。
    applied_wheel_hold_angles_.reset();
}

void RobotIO::applyTargets(const ActuatorTargets& targets) {
    const double left = targets.wheel_speeds.left;
    const double right = targets.wheel_speeds.right;

    // optional 有值表示位置保持，为空表示普通轮速控制
    const bool hold_wheels = targets.wheel_hold_angles.has_value();

    // 1，检查轮速：保持位置时，拒绝同时要求轮子滚动
    if (!std::isfinite(left) || !std::isfinite(right)) {
        throw std::invalid_argument("Wheel speed is not finite");
    }
    if (hold_wheels && (left != 0.0 || right != 0.0)) {
        throw std::invalid_argument("Wheel speed must be zero when holding position");
    }

    // 确认optional有值后，才能通过value()访问
    //const& 避免复制数组，也不允许修改调用者的保持目标。
    if (hold_wheels) {
        const WheelValues& hold_angles = targets.wheel_hold_angles.value();

        for (double angle : hold_angles) {
            if (!std::isfinite(angle)) {
                throw std::invalid_argument("Wheel hold angles must be finite");
            }
        }
    }

    //2. 检查全部腿关节，保留原来的微小越界容差处理。
    //修改副本，不改调用者数据
    LegPoses safe_leg_angles = targets.leg_angles;

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            const double angle = targets.leg_angles[leg_index][joint_index];

            const auto* motor = leg_motors_[leg_index][joint_index];
            const double min_angle = motor->getMinPosition();
            const double max_angle = motor->getMaxPosition();

            if (!std::isfinite(angle) || angle < min_angle - JOINT_LIMIT_EPSILON_RAD
                || angle > max_angle + JOINT_LIMIT_EPSILON_RAD) {
                throw std::out_of_range("Joint target invalid or outside limits: "
                                        + jointName(leg_index, joint_index));
            }

            safe_leg_angles[leg_index][joint_index] = std::clamp(angle, min_angle, max_angle);
        }
    }

    //3. 普通滚动时，左右轮速同比限幅。
    // 保持模式要求 left/right 为零，因此 command 也为零。
    const double scale =
        std::max({1.0, std::abs(left) / max_wheel_speed_, std::abs(right) / max_wheel_speed_});

    const WheelCommand command{left / scale, right / scale};

    // 位置保持时，给位置控制器一个正的纠偏速度上限
    const double hold_speed = std::min(WHEEL_HOLD_MAX_SPEED_RAD_S, max_wheel_speed_);

    if (hold_wheels && (!std::isfinite(hold_speed) || hold_speed <= 0.0)) {
        throw std::invalid_argument("Wheel hold speed limit must be finite and positive");
    }

    //4. 所有输入检查通过后，再发送整租目标。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            leg_motors_[leg_index][joint_index]->setPosition(
                safe_leg_angles[leg_index][joint_index]);
        }

        auto* wheel = wheels_[leg_index];

        if (hold_wheels) {
            //有限位置目标：保持此前保存的轮轴关节角。
            // hold_speed 是纠偏速度上限，不是持续旋转速度。
            wheel->setPosition(targets.wheel_hold_angles.value()[leg_index]);
            wheel->setVelocity(hold_speed);
        } else {
            const bool left_side =
                leg_index == LEFT_FRONT_LEG_INDEX || leg_index == LEFT_HIND_LEG_INDEX;
            const double wheel_speed = left_side ? command.left : command.right;

            //显示恢复连续的速度模式。
            wheel->setPosition(std::numeric_limits<double>::infinity());
            wheel->setVelocity(wheel_speed);
        }
    }  // 结束遍历四条腿的外层 for。

    applied_command_ = command;
    // 输入检查失败时不会走到这里，因此不会把拒绝的目标记成已发送。
    // optional 按值复制：空值会清空旧记录，有值会保存这四个固定角度。
    applied_wheel_hold_angles_ = targets.wheel_hold_angles;
}

void RobotIO::setLegPose(std::size_t leg_index, const JointAngles& angles) {
    if (leg_index >= LEG_COUNT) {
        throw std::out_of_range("Unknown leg index");
    }

    // 全部检查通过后再发出三个命令，避免只更新一部分关节。
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        auto* motor = leg_motors_[leg_index][joint_index];
        const double angle = angles[joint_index];
        if (!std::isfinite(angle) || angle < motor->getMinPosition()
            || angle > motor->getMaxPosition()) {
            throw std::out_of_range("Joint target invalid or outside limits: "
                                    + jointName(leg_index, joint_index));
        }
    }

    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        leg_motors_[leg_index][joint_index]->setPosition(angles[joint_index]);
    }
}

void RobotIO::setLegPose(const std::string& leg_name, const JointAngles& angles) {
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        if (leg_name == LEGS[leg_index]) {
            setLegPose(leg_index, angles);
            return;
        }
    }
    throw std::invalid_argument("Unknown leg: " + leg_name);
}

LegPoses RobotIO::availableJointTorques() const {
    LegPoses available_torques{};
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            available_torques[leg_index][joint_index] =
                leg_motors_[leg_index][joint_index]->getAvailableTorque();
        }
    }
    return available_torques;
}

}  // namespace wl
