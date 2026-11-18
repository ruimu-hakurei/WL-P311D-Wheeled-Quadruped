// 【单腿到位判断实现】三个关节都满足容差才返回 true，供四腿站姿控制调用。
#include "motion.hpp"

#include <cmath>
#include <stdexcept>

namespace wl {

bool jointPoseReached(const JointAngles& measured, const JointAngles& target,
                      double tolerance_rad) {
    if (!std::isfinite(tolerance_rad) || tolerance_rad <= 0.0) {
        throw std::invalid_argument("Tolerance must be finite and positive");
    }

    // 必须三个关节都满足容差；遇到非有限测量直接视为未到位。
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        const double actual_angle = measured[joint_index];
        const double target_angle = target[joint_index];
        if (!std::isfinite(actual_angle) || !std::isfinite(target_angle)) {
            return false;
        }
        if (std::abs(actual_angle - target_angle) > tolerance_rad) {
            return false;
        }
    }
    return true;
}

}  // namespace wl
