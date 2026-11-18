// 【设备边界】封装电机、编码器、姿态/GPS/陀螺仪；算法层只接收值类型。
// 键盘属于入口层的交互输入，不放在这里；设备对象由 Webots 管理。
#pragma once

#include "config.hpp"

#include <array>
#include <optional>
#include <string>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

namespace wl {

// 唯一读写设备的模块。Motor 等对象由 Webots Robot 拥有，本类不 delete 它们。
class RobotIO {
public:
    RobotIO(webots::Robot& robot, int time_step_ms);
    // = delete 禁止拷贝设备封装，避免两个对象持有同一组设备却各自保存不同历史。
    RobotIO(const RobotIO&) = delete;
    RobotIO& operator=(const RobotIO&) = delete;

    // 有效反馈返回 SensorSample；NaN/无穷等无效数据返回 nullopt，而不是伪造全零测量。
    std::optional<SensorSample> read(double now_s);
    void drive(double forward, double turn);
    // 检查四腿关节目标和左右轮速,通过后统一发给电机
    void applyTargets(const ActuatorTargets& targets);
    void setLegPose(std::size_t leg_index, const JointAngles& angles);
    void setLegPose(const std::string& leg_name, const JointAngles& angles);

    WheelCommand appliedCommand() const { return applied_command_; }

    // 返回最近一次下发的保持角度副本，供日志与同一执行区间的反馈配对。
    // 没有值表示轮速控制；这是发送记录，不是编码器测量或实际到位判定。
    std::optional<WheelValues> appliedWheelHoldAngles() const { return applied_wheel_hold_angles_; }

    double maxWheelSpeed() const { return max_wheel_speed_; }

    // 返回电机可用力矩上限，用于运行快照；这不是关节实际输出力矩的测量。
    LegPoses availableJointTorques() const;

private:
    // 初始化分两步：确认所有设备齐全，再设置电机与传感器。
    void findDevices(webots::Robot& robot);
    void initializeDevices(int time_step_ms);

    // read() 的五个步骤；每个步骤承担一种数据处理职责。
    SensorSample readMeasurements(double now_s) const;
    static bool hasValidFeedback(const SensorSample& feedback);
    void estimateSpeeds(SensorSample& feedback) const;
    void updateDisplacements(SensorSample& feedback);
    void updateJointErrors(SensorSample& feedback) const;

    // 指针指向 SDK 对象，使用 -> 调用；数组的 {} 先把句柄初始化为空指针。
    using LegMotors = std::array<webots::Motor*, JOINT_COUNT>;
    using LegEncoders = std::array<webots::PositionSensor*, JOINT_COUNT>;

    std::array<webots::Motor*, LEG_COUNT> wheels_{};
    std::array<webots::PositionSensor*, LEG_COUNT> wheel_encoders_{};
    std::array<LegMotors, LEG_COUNT> leg_motors_{};
    std::array<LegEncoders, LEG_COUNT> leg_encoders_{};
    webots::InertialUnit* imu_ = nullptr;
    webots::GPS* gps_ = nullptr;
    webots::Gyro* gyro_ = nullptr;

    double max_wheel_speed_ = 0.0;
    WheelCommand applied_command_{};

    // 和 applied_command_ 一起保存上次输出；构造时轮子处于速度模式。
    std::optional<WheelValues> applied_wheel_hold_angles_ = std::nullopt;

    // 上一有效采样用于差分；失效时清空时间，恢复后的第一帧不求速度。
    std::optional<double> last_time_s_;
    WheelValues previous_wheel_angles_{};
    LegPoses previous_joint_angles_{};

    std::optional<Vec3> start_world_position_;
    std::optional<std::array<double, 2>> reference_point_origin_;
};

}  // namespace wl
