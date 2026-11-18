// 【机身协调接口】站姿控制已接入主循环；慢速迈步目标组合尚未接入。
// 把阶段计划/轮心目标变成统一 ActuatorTargets，设备执行由 RobotIO 完成。
#pragma once

#include "trajectory.hpp"

#include <optional>

#include <array>

#include "gait.hpp"
#include "kinematics.hpp"

namespace wl {

// 四条腿的全部关节都满足误差要求时，才认为姿态到位。
bool legPosesReached(const LegPoses& measured, const LegPoses& target, double tolerance_rad);

// 四条单腿关节轨迹使用同一个开始时刻，已由 StandController 接入正常主循环。
class StandTransition {
public:
    StandTransition(const LegPoses& measured, const LegPoses& target, double start_time_s,
                    double duration_s);

    LegPoses sample(double now_s) const;

private:
    std::array<JointTransition, LEG_COUNT> transitions_;
};

// 四腿站姿阶段；与早期单腿预览、未来步态状态是三组独立的枚举。
enum class StandPhase {
    Prepare,      // 过渡开始，关节从当前姿态移动到目标姿态
    WaitReached,  // 等待四腿关节到位
    Hold          // 实测到位时保持；受扰后可以回到 WaitReached
};

struct StandGoal {
    StandPhase phase;
    LegPoses target_angles;
    bool reached;
};

// 只生成姿态与判断，不直接访问电机
class StandController {
public:
    StandController(const LegPoses& target_angles, double duration_s, double tolerance_rad);
    // 每拍传入当前时间和实测角度，得到这一拍的控制目标。
    StandGoal update(double now_s, const LegPoses& measured);

private:
    LegPoses target_angles_;  // 最终希望保持的四腿关节角。
    double duration_s_;       // 准备轨迹的持续时间。
    double tolerance_rad_;    // 判断实际到位的角度误差容限。

    // 第一次 update() 才能取得实测起点，因此暂时不创建轨迹。
    std::optional<StandTransition> transition_;

    double start_time_s_ = 0.0;
};

// 参考坐标系取本步开始时的机身坐标系。
// 先处理机身平移，参考系与目标机身坐标系的轴方向保持一致。
struct CrawlStepPlan {
    WheelCenters reference_centers_m{};  // 本步开始时四个轮心的位置。
    Vec3 swing_start_m{};                // 当前摆动腿的起点。
    Vec3 swing_end_m{};                  // 当前摆动腿的落点。
    Vec3 body_shift_m{};                 // 机身相对参考系的目标平移。
    std::size_t active_leg = 0;          // 这份计划对应哪条摆动腿。
    double lift_height_m = 0.0;          // 摆动抬高量。
};

// 每个新步开始时调用一次，冻结起始轮心；逐拍重建会让轨迹起点跟着测量漂移。
CrawlStepPlan makeCrawlStepPlan(const FourLegKinematics& kinematics,
                                const LegPoses& measured_angles, std::size_t active_leg,
                                double step_length_m, double lift_height_m);

// 组合轮心轨迹并求逆解，只生成执行目标。
ActuatorTargets buildCrawlTargets(const GaitGoal& goal, const CrawlStepPlan& step,
                                  const FourLegKinematics& kinematics, const LegJointLimits& limits,
                                  const LegPoses& reference_angles);

// 输入同一坐标系中的四个接地点，计算其余三腿的支撑三角形中心。
// contact_points_m 是地面接触位置，单位 m。
Vec3 supportTriangleCenter(const std::array<Vec3, LEG_COUNT>& contact_points_m,
                           std::size_t swing_leg);
double signedSupportMarginXY(const std::array<Vec3, 3>& triangle,
                             const Vec3& com_projection);
// 估算排除摆动腿后，质心投影到支撑三角形边缘的有符号裕量，单位 m。
double estimateSupportMargin(const FourLegKinematics& kinematics,
                             const SensorSample& feedback,
                             std::size_t swing_leg);

// 计算支撑准备所需的机身平移，单位 m。
// step 与 start_feedback 必须来自同一个步起始快照。
// 只返回规划目标，不发送电机命令。
// 使用约定：
//   kinematics：四条腿的固定几何；limits：每条腿三个关节的角度范围，单位 rad。
//   step：在本步开始时冻结的轮心参考；start_feedback：同一时刻的关节/轮子角与姿态。
//   调用者负责保证这两个输入快照一致；函数签名本身不能检查它们是否同时采集。
// 返回值：本步起始参考系中的机身平移，不是绝对世界位置，也不是新的电机角度。
// 规划假设：水平地面、机身方向保持不变，轮心参考固定；内部不推进仿真时间。
// 失败通过异常报告：退化支撑三角形、候选逆解不可达或迭代未收敛都不返回执行目标。
// 收敛只证明候选姿态的模型投影误差足够小，不能直接作为 support_ready 的实测依据。
// const T& 表示只读借用输入；Vec3 按值返回，调用者保存结果后再决定何时平滑执行。
Vec3 planSupportShift(const FourLegKinematics& kinematics, const CrawlStepPlan& step,
                      const SensorSample& start_feedback, const LegJointLimits& limits);

// 抵消腿部关节绕轮轴方向的自转，返回四个轮电机的位置目标。
WheelValues compensateWheelAngles(
    const LegPoses& start_leg_angles,
    const WheelValues& start_wheel_angles,
    const LegPoses& target_leg_angles);

// 在 3 秒内从零平移到 start_step.body_shift_m，并保持终点目标。
ActuatorTargets sampleBodyShiftTargets(
    const FourLegKinematics& kinematics,
    const CrawlStepPlan& start_step,
    const SensorSample& start_feedback,
    const LegJointLimits& limits,
    double elapsed_s);

}  // namespace wl
