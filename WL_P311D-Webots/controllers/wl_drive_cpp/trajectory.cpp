// 【轨迹算法】时间先归一化，再通过五次时间标度生成平滑目标。
// 轨迹给出期望值；实际电机能否跟上，需要反馈和执行层另外判断。
#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wl {
namespace {

bool isFiniteVector(const Vec3& vector) {
    for (double component : vector) {
        if (!std::isfinite(component)) {
            return false;
        }
    }
    return true;
}

}  // namespace

double quinticProgress(double elapsed_s, double duration_s) {
    const bool valid_time = std::isfinite(elapsed_s) && std::isfinite(duration_s);
    if (!valid_time || duration_s <= 0.0) {
        throw std::invalid_argument(
            "Elapsed time must be finite; duration must be finite and positive");
    }

    // 归一化时间限制到 [0,1]，开始前保持起点，结束后保持终点。
    const double normalized_time = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    // 五次曲线在起终点的一阶、二阶导数都为零，使目标速度和加速度平滑起落。
    return 10.0 * std::pow(normalized_time, 3) - 15.0 * std::pow(normalized_time, 4)
           + 6.0 * std::pow(normalized_time, 5);
}

Vec3 interpolatePosition(const Vec3& start, const Vec3& end, double elapsed_s, double duration_s) {
    if (!isFiniteVector(start) || !isFiniteVector(end)) {
        throw std::invalid_argument("Positions must be finite");
    }

    const double progress = quinticProgress(elapsed_s, duration_s);
    Vec3 position{};
    for (std::size_t axis = 0; axis < position.size(); ++axis) {
        position[axis] = start[axis] + progress * (end[axis] - start[axis]);
    }
    return position;
}

Vec3 swingPosition(const Vec3& start, const Vec3& end, double progress, double lift_height_m) {
    // 抬高量必须是有限的非负数。
    if (!std::isfinite(lift_height_m) || lift_height_m < 0.0) {
        throw std::invalid_argument("Lift height must be finite and nonnegative");
    }

    // 先计算从起点到终点的平滑移动。
    // 这里使用归一化进度，实际摆动时长仍由 CrawlGait 决定。
    // 这里的 1.0 是归一化尺度，不是把真实摆动时间固定成一秒；progress 已由外层计时。
    Vec3 position = interpolatePosition(start, end, progress, 1.0);

    // 复用五次平滑函数，得到起止变化平缓的进度。
    const double s = quinticProgress(progress, 1.0);

    // 在直线轨迹上叠加抬高：
    // 起点为零，中点达到 lift_height_m，终点回到零。
    // 抬高沿当前坐标系 +Z；只有该轴近似竖直时才能直接理解为离地高度。
    position[2] += 4.0 * lift_height_m * s * (1.0 - s);

    return position;
}

// 构造函数后面的 : 初始化各成员；sample() 只读取这些状态，不重置轨迹起点。
JointTransition::JointTransition(JointAngles start_angles, JointAngles end_angles,
                                 double start_time_s, double duration_s)
    : start_angles_(start_angles), end_angles_(end_angles), start_time_s_(start_time_s),
      duration_s_(duration_s) {
    if (!isFiniteVector(start_angles_) || !isFiniteVector(end_angles_)
        || !std::isfinite(start_time_s_)) {
        throw std::invalid_argument("Joint transition inputs must be finite");
    }
    quinticProgress(0.0, duration_s_);  // 复用时间参数检查。
}

JointAngles JointTransition::sample(double now_s) const {
    const double elapsed_s = now_s - start_time_s_;
    const double progress = quinticProgress(elapsed_s, duration_s_);

    JointAngles target_angles{};
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        const double start_angle = start_angles_[joint_index];
        const double end_angle = end_angles_[joint_index];
        target_angles[joint_index] = start_angle + progress * (end_angle - start_angle);
    }
    return target_angles;
}

}  // namespace wl
