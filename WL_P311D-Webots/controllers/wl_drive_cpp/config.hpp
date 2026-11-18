// 【参数层】集中保存设备顺序、URDF 几何/质量、轨迹参数和执行容差。
// inline constexpr 是 C++17 头文件常量写法：允许多个源文件包含同一份定义。
#pragma once

#include "models.hpp"

#include <array>

namespace wl {

// 全项目统一的数组索引顺序。
inline constexpr std::array<const char*, LEG_COUNT> LEGS = {"LF", "RF", "LH", "RH"};
inline constexpr std::array<const char*, JOINT_COUNT> LEG_JOINTS = {"HAA", "HFE", "KFE"};
inline constexpr std::size_t LEFT_FRONT_LEG_INDEX = 0;
inline constexpr std::size_t LEFT_HIND_LEG_INDEX = 2;

// 轮速分配量，单位 rad/s；不是车身线速度或偏航角速度。
inline constexpr double FORWARD_SPEED = 10.0;
inline constexpr double TURN_SPEED = 5.0;
inline constexpr double WHEEL_ACCELERATION = 4.0;  // rad/s²。

inline constexpr double SETTLE_SECONDS = 2.0;
inline constexpr double PRINT_PERIOD_SECONDS = 0.5;

// 零位轮心几何定义的机身固定参考点，单位 m，不是质心。
inline constexpr double REFERENCE_CENTER_X_M = -0.11101;

// 位置保持时的纠偏速度上限，单位 rad/s。
// 这是初始设置，实际保持效果后续通过仿真观察。
inline constexpr double WHEEL_HOLD_MAX_SPEED_RAD_S = 1.0;

// 按 H 进入位置保持前，四轮实测角速度都必须低于此阈值。
// 这是本次站姿验证的轮速条件，不代表已检测到轮地接触或机身绝对静止。
inline constexpr double WHEEL_HOLD_ENTRY_SPEED_TOLERANCE_RAD_S = 0.05;

// B 键支撑平移时长，由轨迹采样和到位日志共同使用。
inline constexpr double BODY_SHIFT_DURATION_S = 3.0;

// 左前腿几何：机身到髋、髋到大腿、大腿到膝、膝到轮心，单位 m。
inline constexpr LegGeometry LF_GEOMETRY{{0.216, 0.060, -0.07376},
                                         {0.058, 0.04080, 0.0},
                                         {-0.280, 0.08155, 0.0},
                                         {0.21449, 0.03434, -0.17998}};

// 右前腿：四个向量依次对应机身到髋、髋到大腿、大腿到膝、膝到轮心。
// 单位 m，定义方式与 LF_GEOMETRY 相同。
inline constexpr LegGeometry RF_GEOMETRY{{0.216, -0.060, -0.07376},
                                         {0.058, -0.04080, 0.0},
                                         {-0.280, -0.08155, 0.0},
                                         {0.21449, -0.03434, -0.17998}};

// 左后腿。
inline constexpr LegGeometry LH_GEOMETRY{{-0.307, 0.060, -0.07376},
                                         {-0.058, 0.04080, 0.0},
                                         {-0.280, 0.08155, 0.0},
                                         {0.21449, 0.03434, -0.17998}};

// 右后腿。
inline constexpr LegGeometry RH_GEOMETRY{{-0.307, -0.060, -0.07376},
                                         {-0.058, -0.04080, 0.0},
                                         {-0.280, -0.08155, 0.0},
                                         {0.21449, -0.03434, -0.17998}};

// 按项目统一索引保存四腿几何。
inline constexpr std::array<LegGeometry, LEG_COUNT> LEG_GEOMETRIES{LF_GEOMETRY, RF_GEOMETRY,
                                                                   LH_GEOMETRY, RH_GEOMETRY};

// 每个关节的下限、上限，单位 rad。
inline constexpr JointLimits LF_JOINT_LIMITS{
    {{-0.87, 0.87}, {-2.27, 1.22}, {0.0, 2.0943951023932}}};
// 右前腿：HAA、HFE、KFE 的下限与上限，单位 rad。
inline constexpr JointLimits RF_JOINT_LIMITS{
    {{-0.87, 0.87}, {-2.27, 1.22}, {0.0, 2.0943951023932}}};

// 左后腿。
inline constexpr JointLimits LH_JOINT_LIMITS{
    {{-0.87, 0.87}, {-2.27, 1.22}, {0.0, 2.0943951023932}}};

// 右后腿。
inline constexpr JointLimits RH_JOINT_LIMITS{
    {{-0.87, 0.87}, {-2.27, 1.22}, {0.0, 2.0943951023932}}};

// 四腿限位表，顺序始终为 LF、RF、LH、RH。
inline constexpr LegJointLimits LEG_JOINT_LIMITS{LF_JOINT_LIMITS, RF_JOINT_LIMITS, LH_JOINT_LIMITS,
                                                 RH_JOINT_LIMITS};

// 以下步态参数已经定义，但当前 runController 尚未创建 CrawlGait 或执行迈步。
// 首版慢速步态的轨迹参数。
inline constexpr double CRAWL_SWING_DURATION_S = 2.0;
inline constexpr double CRAWL_STEP_LENGTH_M = 0.04;
inline constexpr double CRAWL_LIFT_HEIGHT_M = 0.02;

// 四腿姿态准备目标。
// 行顺序：LF、RF、LH、RH；列顺序：HAA、HFE、KFE。
// 角度单位 rad；当前由 StandController 插值并交给 RobotIO 执行。
inline constexpr LegPoses STAND_TARGET_ANGLES{{
    {0.0, -0.4, 0.5},  // 左前腿
    {0.0, -0.4, 0.5},  // 右前腿
    {0.0, -0.4, 0.5},  // 左后腿
    {0.0, -0.4, 0.5}   // 右后腿
}};

inline constexpr double STAND_DURATION_S = 3.0;
inline constexpr double STAND_TOLERANCE_RAD = 0.01;

// 执行目标的微小越界容差，不是判断姿态到位的误差容限。
inline constexpr double JOINT_LIMIT_EPSILON_RAD = 1e-4;

// 质量单位 kg，局部质心单位 m；同一部件的质心不一定恰好位于关节原点。
// 计算整机质心时，腿部这些局部坐标必须先通过 pointToBase 转换，不能直接相加。
// 根部件：base。
inline constexpr LinkMass BASE_MASS{18.188, {-0.042212, -0.000883, -0.085535}};

// 当前 IMU 固定关节的平移、旋转均为零。
inline constexpr LinkMass IMU_MASS{0.01, {0.0, 0.0, 0.0}};

// 每条腿内部顺序：Hip、Thigh、Calf、Wheel。
inline constexpr LegMasses LF_LINK_MASSES{{{1.434, {0.053246, 0.009578, -0.000256}},
                                           {2.751, {-0.057303, 0.055844, 0.004528}},
                                           {1.326, {0.172578, 0.002711, -0.142172}},
                                           {0.818, {0.000038, 0.000878, 0.000051}}}};

inline constexpr LegMasses RF_LINK_MASSES{{{1.434, {0.053246, -0.009578, -0.000256}},
                                           {2.751, {-0.057303, -0.055844, 0.004528}},
                                           {1.326, {0.172578, -0.002711, -0.142172}},
                                           {0.818, {0.000038, -0.000878, 0.000051}}}};

inline constexpr LegMasses LH_LINK_MASSES{{{1.434, {-0.053246, 0.009578, -0.000256}},
                                           {2.751, {-0.057303, 0.055844, 0.004528}},
                                           {1.326, {0.172578, 0.002711, -0.142172}},
                                           {0.818, {0.000038, 0.000878, 0.000051}}}};

inline constexpr LegMasses RH_LINK_MASSES{{{1.434, {-0.053246, -0.009578, -0.000256}},
                                           {2.751, {-0.057303, -0.055844, 0.004528}},
                                           {1.326, {0.172578, -0.002711, -0.142172}},
                                           {0.818, {0.000038, -0.000878, 0.000051}}}};

// 四腿顺序：LF、RF、LH、RH。
inline constexpr std::array<LegMasses, LEG_COUNT> LEG_LINK_MASSES{LF_LINK_MASSES, RF_LINK_MASSES,
                                                                  LH_LINK_MASSES, RH_LINK_MASSES};
// 两个轮子尺寸常量
inline constexpr double WHEEL_RADIUS_M = 0.127;
inline constexpr double WHEEL_WIDTH_M = 0.017;

}  // namespace wl
