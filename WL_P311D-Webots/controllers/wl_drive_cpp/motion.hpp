// 【单腿到位判断】检查一条腿的三个关节是否都到达目标。
// 由四腿站姿控制复用；不访问设备，也不检测接触或支撑稳定性。
#pragma once

#include "models.hpp"

namespace wl {

// measured/target 顺序为 HAA、HFE、KFE，角度与正容差均使用 rad。
bool jointPoseReached(const JointAngles& measured, const JointAngles& target, double tolerance_rad);

}  // namespace wl
