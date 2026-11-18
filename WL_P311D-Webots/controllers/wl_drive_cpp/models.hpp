// 【数据契约】只定义各层交换的数据，不访问设备，也不计算控制量。
// 先确认“是什么数据、哪个坐标系、什么单位”，再读使用它的算法。
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <optional>

namespace wl {

inline constexpr std::size_t LEG_COUNT = 4;
inline constexpr std::size_t JOINT_COUNT = 3;

// 所有四腿数组都按 LF、RF、LH、RH 排列；关节按 HAA、HFE、KFE 排列。
// using 是类型别名，不会创建对象；std::array<T,N> 的元素个数在编译期固定。
// Vec3 和 JointAngles 虽然物理意义不同，当前底层类型相同，仍须靠接口和单位区分。
// LF/RF/LH/RH = 左前/右前/左后/右后；HAA/HFE/KFE = 髋侧摆/髋俯仰/膝屈伸。
using Vec3 = std::array<double, 3>;
using JointAngles = std::array<double, JOINT_COUNT>;
// 二维数组：leg_angles[腿编号][关节编号]，四条腿 × 每腿三个腿关节，不含轮电机。
using LegPoses = std::array<JointAngles, LEG_COUNT>;
using WheelValues = std::array<double, LEG_COUNT>;
using JointLimits = std::array<std::array<double, 2>, JOINT_COUNT>;
// 四个轮心的位置，顺序 LF、RF、LH、RH。
// 每个位置都表达在机身坐标系中，单位 m。
using WheelCenters = std::array<Vec3, LEG_COUNT>;

// 四条腿各自的三个关节限位。
using LegJointLimits = std::array<JointLimits, LEG_COUNT>;

// 每个向量：子关节原点在父坐标系中的位置，单位 m。
struct LegGeometry {
    Vec3 base_to_hip;
    Vec3 hip_to_thigh;
    Vec3 thigh_to_knee;
    Vec3 knee_to_wheel;
};

struct WheelCommand {
    double left = 0.0;   // 左侧两轮的目标角速度，rad/s。
    double right = 0.0;  // 右侧两轮的目标角速度，rad/s。
};

// 统一执行目标：控制层生成数据，RobotIO 检查后发送。
struct ActuatorTargets {
    // 四腿目标角度，顺序 LF、RF、LH、RH。
    // 每腿三个关节：HAA、HFE、KFE，单位 rad。
    LegPoses leg_angles;

    // 普通滚动时使用的左右轮目标角速度，单位 rad/s。
    WheelCommand wheel_speeds;

    // 可选的四轮保持角度，顺序 LF、RF、LH、RH，单位 rad。
    //
    // 没有值：按照 wheel_speeds 控制轮速。
    // 存在值：按照这四个固定角度进行位置保持；
    //         此时要求 wheel_speeds 为零，避免两种意图冲突。
    //
    // 默认 nullopt 表示没有保持目标，保留原来的轮式控制行为。
    std::optional<WheelValues> wheel_hold_angles = std::nullopt;

    // 必须明确提供四腿目标，轮速可以省略。
    //
    // explicit：避免把 LegPoses 意外隐式转换成执行目标。
    // const&：只读借用输入，构造时再复制到自己的成员中。
    explicit ActuatorTargets(
        const LegPoses& target_leg_angles,
        WheelCommand target_wheel_speeds = {})
        : leg_angles(target_leg_angles),
          wheel_speeds(target_wheel_speeds) {
    }
};

struct MotionCommand {
    // 保留 Python 接口：两者都是轮速分配量，单位 rad/s。
    // 例如 forward=0、turn>0 时左右轮目标反向；是否原地转动仍受接触物理影响。
    double forward = 0.0;
    double turn = 0.0;
};

// 同一仿真时刻的测量与派生量，不保存设备指针。
struct SensorSample {
    double time_s = 0.0;

    // 机身姿态，rad。
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;

    // 编码器角度 rad，差分速度 rad/s。
    // {} 对数值数组做零初始化；这些零只是初值，是否有效仍由 read() 和 speeds_valid 判断。
    WheelValues angles{};
    WheelValues speeds{};
    LegPoses joint_angles{};
    LegPoses joint_targets{};
    LegPoses joint_errors{};  // 实际电机目标减去实测角度。
    LegPoses joint_speeds{};

    // 第一帧、无效反馈恢复后的第一帧没有差分速度，CSV 对应列留空。
    bool speeds_valid = false;

    Vec3 position{};                // GPS，世界坐标系，m。
    Vec3 world_velocity{};          // GPS 速度，世界坐标系，m/s。
    Vec3 angular_velocity{};        // 陀螺仪，设备坐标系，rad/s。
    Vec3 lf_wheel_center_base_m{};  // FK 轮心位置，机身坐标系，m。

    double horizontal_displacement_m = 0.0;
    double reference_center_x_m = 0.0;
    double reference_center_y_m = 0.0;
    double reference_center_displacement_m = 0.0;

    double max_joint_error_rad = 0.0;
    std::string worst_joint;
};

// 一个部件的质量，以及它的质心在机身坐标系中的位置。
struct MassPoint {
    double mass_kg = 0.0;
    Vec3 position_base_m{};
};

// 每条腿的四个带质量部件。
// enum class 是带作用域的枚举，要写 LegLink::Wheel，避免 Hip 等名字污染命名空间。
enum class LegLink { Hip = 0, Thigh = 1, Calf = 2, Wheel = 3 };

inline constexpr std::size_t LEG_LINK_COUNT = 4;

// 来自 URDF 的部件质量和局部质心。
struct LinkMass {
    double mass_kg = 0.0;
    Vec3 center_local_m{};
};

using LegMasses = std::array<LinkMass, LEG_LINK_COUNT>;

}  // namespace wl
