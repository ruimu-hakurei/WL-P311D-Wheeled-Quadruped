// 【运动学实现】当前模型关节轴依次为 X、Y、Y，安装坐标系零位旋转为零。
// 位置默认在 base 机身坐标系中，单位 m；关节角单位 rad。
// 几何候选解 → 限位筛选 → 正解回代 → 参考姿态选解，是完整逆解链。
#include "kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wl {
namespace {

// 以下容差用于数值几何/分支判断，区别于 config.hpp 的实际执行限位容差。
constexpr double GEOMETRY_TOLERANCE = 1e-12;
constexpr double COSINE_TOLERANCE = 1e-10;
constexpr double ANGLE_TOLERANCE_RAD = 1e-9;
constexpr double POSITION_TOLERANCE_M = 1e-7;

bool isFiniteVector(const Vec3& vector) {
    for (double component : vector) {
        if (!std::isfinite(component)) {
            return false;
        }
    }
    return true;
}

Vec3 addVectors(const Vec3& first, const Vec3& second) {
    return {first[0] + second[0], first[1] + second[1], first[2] + second[2]};
}

Vec3 rotateAroundX(const Vec3& vector, double angle_rad) {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return {vector[0], cosine * vector[1] - sine * vector[2],
            sine * vector[1] + cosine * vector[2]};
}

Vec3 rotateAroundY(const Vec3& vector, double angle_rad) {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return {cosine * vector[0] + sine * vector[2], vector[1],
            -sine * vector[0] + cosine * vector[2]};
}

// 把相差整圈的角度归到主值区间；不是给目标强行做电机限位裁剪。
double wrapAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double positionDistance(const Vec3& first, const Vec3& second) {
    return std::hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}

bool sameAngles(const JointAngles& first, const JointAngles& second) {
    double largest_difference = 0.0;
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        const double difference = std::abs(first[joint_index] - second[joint_index]);
        largest_difference = std::max(largest_difference, difference);
    }
    return largest_difference < ANGLE_TOLERANCE_RAD;
}

bool withinJointLimits(const JointAngles& angles, const JointLimits& limits) {
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        const double lower = limits[joint_index][0] - ANGLE_TOLERANCE_RAD;
        const double upper = limits[joint_index][1] + ANGLE_TOLERANCE_RAD;
        const bool within = angles[joint_index] >= lower && angles[joint_index] <= upper;
        if (!within) {
            return false;
        }
    }
    return true;
}

double squaredAngleDistance(const JointAngles& angles, const JointAngles& reference) {
    double distance_squared = 0.0;
    for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
        const double difference = angles[joint_index] - reference[joint_index];
        distance_squared += difference * difference;
    }
    return distance_squared;
}

}  // namespace

LegKinematics::LegKinematics(LegGeometry geometry) : geometry_(geometry) {}

Vec3 LegKinematics::forward(double haa, double hfe, double kfe) const {
    return forward(JointAngles{haa, hfe, kfe});
}

Vec3 LegKinematics::forward(const JointAngles& angles) const {
    if (!isFiniteVector(angles)) {
        throw std::invalid_argument("Joint angles must be finite");
    }

    // 从轮心开始，按膝关节、髋俯仰、髋侧摆的顺序逐级变换到机身。
    // 从末端向根部计算，是把局部位置逐级表达在父坐标系中；不是倒着控制关节。
    // 轮心位于轮子自身原点，因此轮电机自转角不会改变 forward() 的轮心位置。
    Vec3 position = rotateAroundY(geometry_.knee_to_wheel, angles[2]);
    position = addVectors(geometry_.thigh_to_knee, position);
    position = rotateAroundY(position, angles[1]);
    position = addVectors(geometry_.hip_to_thigh, position);
    position = rotateAroundX(position, angles[0]);
    return addVectors(geometry_.base_to_hip, position);
}

std::vector<JointAngles> LegKinematics::inverseCandidates(const Vec3& target) const {
    if (!isFiniteVector(target)) {
        throw std::invalid_argument("Target must be finite");
    }

    const auto& base_offset = geometry_.base_to_hip;
    const auto& hip_offset = geometry_.hip_to_thigh;
    const auto& thigh_offset = geometry_.thigh_to_knee;
    const auto& calf_offset = geometry_.knee_to_wheel;

    // 1. 去掉机身到侧摆关节的安装偏移。
    const double x = target[0] - base_offset[0];
    const double y = target[1] - base_offset[1];
    const double z = target[2] - base_offset[2];

    // 2. Y 轴旋转保持侧向偏移不变，先解出侧摆平面里的两个高度分支。
    const double lateral_offset = hip_offset[1] + thigh_offset[1] + calf_offset[1];
    const double height_squared = y * y + z * z - lateral_offset * lateral_offset;
    if (height_squared < -GEOMETRY_TOLERANCE) {
        return {};
    }
    const double height = std::sqrt(std::max(0.0, height_squared));

    // 3. 提取 XZ 平面中的连杆长度与零位方向，保持原 Python 的符号约定。
    const double thigh_length = std::hypot(thigh_offset[0], thigh_offset[2]);
    const double calf_length = std::hypot(calf_offset[0], calf_offset[2]);
    if (thigh_length <= 0.0 || calf_length <= 0.0) {
        throw std::invalid_argument("Link lengths must be positive");
    }
    const double thigh_zero_angle = std::atan2(-thigh_offset[2], thigh_offset[0]);
    const double calf_zero_angle = std::atan2(-calf_offset[2], calf_offset[0]);

    std::vector<JointAngles> candidates;
    // 外层两个侧摆分支 × 内层两个膝弯曲分支，最多生成四组几何候选。
    // 几何有解不代表电机可达，因此这里不直接下结论，后面 inverse() 还会筛选。
    for (double hip_z : {-height, height}) {
        const double haa = wrapAngle(std::atan2(z, y) - std::atan2(hip_z, lateral_offset));
        const double planar_x = x - hip_offset[0];
        const double planar_z = hip_z - hip_offset[2];

        // 4. 余弦定理求两连杆夹角。只修正浮点误差引起的微小越界。
        const double cosine_delta = (planar_x * planar_x + planar_z * planar_z
                                     - thigh_length * thigh_length - calf_length * calf_length)
                                    / (2.0 * thigh_length * calf_length);
        if (cosine_delta < -1.0 - COSINE_TOLERANCE || cosine_delta > 1.0 + COSINE_TOLERANCE) {
            continue;
        }
        const double delta = std::acos(std::clamp(cosine_delta, -1.0, 1.0));

        // 5. 每个侧摆分支中，再枚举膝关节的两个弯曲方向。
        for (double bend : {-delta, delta}) {
            const double kfe = wrapAngle(thigh_zero_angle - calf_zero_angle + bend);
            const double cosine = std::cos(kfe);
            const double sine = std::sin(kfe);

            // 固定膝角后，把两段连杆合成一条“虚拟连杆”，其方向与目标方向之差给出 HFE。
            const double virtual_x =
                thigh_offset[0] + cosine * calf_offset[0] + sine * calf_offset[2];
            const double virtual_z =
                thigh_offset[2] - sine * calf_offset[0] + cosine * calf_offset[2];
            const double hfe =
                wrapAngle(std::atan2(virtual_z, virtual_x) - std::atan2(planar_z, planar_x));
            const JointAngles candidate{haa, hfe, kfe};

            // 分支在边界处可能重合，重复解只保留一次。
            // lambda 是临时小函数；[&] 通过引用使用外层 candidate，any_of 检查是否已存在。
            const bool duplicate =
                std::any_of(candidates.begin(), candidates.end(), [&](const JointAngles& previous) {
                    return sameAngles(candidate, previous);
                });
            if (!duplicate) {
                candidates.push_back(candidate);
            }
        }
    }
    return candidates;
}

JointAngles LegKinematics::inverse(const Vec3& target, const JointLimits& limits,
                                   const JointAngles& reference) const {
    for (const auto& bound : limits) {
        if (!std::isfinite(bound[0]) || !std::isfinite(bound[1]) || bound[0] > bound[1]) {
            throw std::invalid_argument("Invalid joint limits");
        }
    }
    if (!isFiniteVector(reference)) {
        throw std::invalid_argument("Reference must be finite");
    }

    std::vector<JointAngles> valid_solutions;
    for (auto angles : inverseCandidates(target)) {
        if (!withinJointLimits(angles, limits)) {
            continue;
        }

        // 这里只修正已通过限位筛选的微小浮点越界，不裁剪真正不可达的目标。
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            angles[joint_index] =
                std::clamp(angles[joint_index], limits[joint_index][0], limits[joint_index][1]);
        }
        // 逆解后再算一次正解：既检查符号/分支，也保证微小限位修正没有破坏位置精度。
        if (positionDistance(forward(angles), target) > POSITION_TOLERANCE_M) {
            continue;
        }
        valid_solutions.push_back(angles);
    }

    if (valid_solutions.empty()) {
        throw std::domain_error("Target unreachable or no solution within joint limits");
    }

    // 从合法解中选离参考姿态最近的一组，减少切换逆解分支造成的跳变。
    // min_element 返回迭代器，*nearest 才是选中的关节数组；按平方距离选解无需开方。
    // 参考角通常取实测值或上拍解，可以减少多解间跳变，但不保证穿越奇异点也连续。
    const auto nearest = std::min_element(valid_solutions.begin(), valid_solutions.end(),
                                          [&](const JointAngles& first, const JointAngles& second) {
                                              return squaredAngleDistance(first, reference)
                                                     < squaredAngleDistance(second, reference);
                                          });
    return *nearest;
}

FourLegKinematics::FourLegKinematics(const std::array<LegGeometry, LEG_COUNT>& geometries)
    : legs_{LegKinematics(geometries[0]), LegKinematics(geometries[1]),
            LegKinematics(geometries[2]), LegKinematics(geometries[3])} {
    // 按 LF、RF、LH、RH 的顺序创建四个单腿运动学对象。
}

WheelCenters FourLegKinematics::forward(const LegPoses& angles) const {
    WheelCenters positions{};

    // 每条腿使用自己的几何参数计算轮心位置。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        positions[leg_index] = legs_[leg_index].forward(angles[leg_index]);
    }

    return positions;
}

LegPoses FourLegKinematics::inverse(const WheelCenters& targets, const LegJointLimits& limits,
                                    const LegPoses& reference) const {
    LegPoses angles{};

    // 分别求解四条腿，并使用各自的限位和参考角度。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        angles[leg_index] =
            legs_[leg_index].inverse(targets[leg_index], limits[leg_index], reference[leg_index]);
    }

    return angles;
}

Vec3 LegKinematics::pointToBase(LegLink link, const Vec3& local_point, const JointAngles& angles,
                                double wheel_angle_rad) const {
    if (!isFiniteVector(local_point) || !isFiniteVector(angles)
        || !std::isfinite(wheel_angle_rad)) {
        throw std::invalid_argument("Point, angles and wheel angle must be finite");
    }

    if (link < LegLink::Hip || link > LegLink::Wheel) {
        throw std::invalid_argument("Invalid leg link");
    }

    // 连杆局部点与轮心不同：轮上的偏心点/局部质心会受 wheel_angle_rad 影响。
    // 各 if 按点所属连杆选择需要经过的关节链，故意不是互斥的 else if。
    Vec3 position = local_point;

    // 1. 轮子局部点： 经过轮子转动，进入小腿坐标系
    if (link == LegLink::Wheel) {
        position = rotateAroundY(position, wheel_angle_rad);
        position = addVectors(geometry_.knee_to_wheel, position);
    }

    // 2. 小腿局部点： 经过膝关节转动，进入大腿坐标系
    if (link == LegLink::Calf || link == LegLink::Wheel) {
        position = rotateAroundY(position, angles[2]);
        position = addVectors(geometry_.thigh_to_knee, position);
    }

    // 3. 大腿局部点： 经过髋俯仰关节转动，进入髋侧摆坐标系
    if (link != LegLink::Hip) {
        position = rotateAroundY(position, angles[1]);
        position = addVectors(geometry_.hip_to_thigh, position);
    }

    // 4. 所有连杆的点最终都经过髋侧摆关节转动，进入机身坐标系
    position = rotateAroundX(position, angles[0]);
    return addVectors(geometry_.base_to_hip, position);
}

Vec3 FourLegKinematics::pointToBase(std::size_t leg_index, LegLink link, const Vec3& local_point,
                                    const LegPoses& angles, const WheelValues& wheel_angles) const {
    if (leg_index >= LEG_COUNT) {
        throw std::out_of_range("Invalid leg index");
    }

    return legs_[leg_index].pointToBase(link, local_point, angles[leg_index],
                                        wheel_angles[leg_index]);
}

}  // namespace wl
