// 【模型估计实现】先把各部件局部质心统一到 base，再计算整机质心。
// 接地点算法假设已知地面法向，返回几何最低点；轮子悬空时也能算出这个点。
#include "estimation.hpp"

#include <cmath>
#include <stdexcept>
#include "config.hpp"

namespace wl {
Vec3 computeCenterOfMass(const std::vector<MassPoint>& points) {
    double total_mass = 0.0;
    Vec3 weighted_position{};

    // 1. 累加总质量,以及质量乘以位置。
    for (const auto& point : points) {
        if (!std::isfinite(point.mass_kg) || point.mass_kg < 0.0) {
            throw std::invalid_argument("Mass must be finite and non-negative");
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double position = point.position_base_m[axis];

            if (!std::isfinite(point.mass_kg) || !std::isfinite(position)) {
                throw std::invalid_argument("Mass and position must be finite");
            }

            weighted_position[axis] += point.mass_kg * position;
        }

        total_mass += point.mass_kg;
    }

    // 空集合或全部质量为零时，质心无法定义。
    if (!std::isfinite(total_mass) || total_mass <= 0.0) {
        throw std::invalid_argument("Total mass must be finite and positive");
    }

    // 2. 计算x,y,z质心位置
    Vec3 center{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        center[axis] = weighted_position[axis] / total_mass;

        if (!std::isfinite(center[axis])) {
            throw std::runtime_error("Computed center of mass is not finite");
        }
    }

    return center;
}

Vec3 estimateCenterOfMass(const FourLegKinematics& kinematics, const LegPoses& joint_angles,
                          const WheelValues& wheel_angles) {
    std::vector<MassPoint> points;

    // 机身,IMU,加上四个腿的各个连杆，共18个质量点。
    // vector 是动态数组；reserve 只预留容量，不增加元素个数，push_back 才加入数据。
    points.reserve(2 + LEG_COUNT * LEG_LINK_COUNT);

    // 机身自身的局部坐标系是base。
    points.push_back(MassPoint{BASE_MASS.mass_kg, BASE_MASS.center_local_m});

    // 当前模型中，IMU 固定关节的平移和旋转均为零。
    points.push_back(MassPoint{IMU_MASS.mass_kg, IMU_MASS.center_local_m});

    // 将四条腿各部件的局部质心转换到 base。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t link_index = 0; link_index < LEG_LINK_COUNT; ++link_index) {
            const LinkMass& mass = LEG_LINK_MASSES[leg_index][link_index];

            // 将数组下标 0～3 转换成 Hip、Thigh、Calf、Wheel。
            // enum class 不会从整数隐式转换；这里已由循环保证编号在 0～3 内。
            const LegLink link = static_cast<LegLink>(link_index);

            const Vec3 center_base_m = kinematics.pointToBase(leg_index, link, mass.center_local_m,
                                                              joint_angles, wheel_angles);

            points.push_back(MassPoint{mass.mass_kg, center_base_m});
        }
    }

    // estimateCenterOfMass 是基于模型的几何计算：没有使用接触力反推质心，也没有滤波。
    // 所有位置已在同一个坐标系中，可以按质量加权。
    return computeCenterOfMass(points);
}

Vec3 estimateWheelContactPoint(const Vec3& wheel_center, const Vec3& wheel_axis,
                               const Vec3& ground_normal, double radius_m, double width_m) {
    constexpr double EPSILON = 1e-12;

    if (!std::isfinite(radius_m) || radius_m <= 0.0 || !std::isfinite(width_m) || width_m < 0.0) {
        throw std::invalid_argument("Invalid wheel dimensions");
    }

    for (std::size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(wheel_center[i]) || !std::isfinite(wheel_axis[i])
            || !std::isfinite(ground_normal[i])) {
            throw std::invalid_argument("Wheel geometry must be finite");
        }
    }

    // 1. 将轮轴方向和地面法向转换为单位向量。
    const double axis_norm = std::hypot(wheel_axis[0], wheel_axis[1], wheel_axis[2]);
    const double normal_norm = std::hypot(ground_normal[0], ground_normal[1], ground_normal[2]);

    if (!std::isfinite(axis_norm) || axis_norm <= EPSILON || !std::isfinite(normal_norm)
        || normal_norm <= EPSILON) {
        throw std::invalid_argument("Invalid wheel axis or ground normal");
    }

    Vec3 axis{};
    Vec3 normal{};

    for (std::size_t i = 0; i < 3; ++i) {
        axis[i] = wheel_axis[i] / axis_norm;
        normal[i] = ground_normal[i] / normal_norm;
    }

    // 2. 把地面法向分解为沿轮轴和垂直轮轴的两个分量。
    // 点积给出法向沿轮轴的投影；减去这部分后，剩余方向位于轮子的圆截面。
    const double axial_component = normal[0] * axis[0] + normal[1] * axis[1] + normal[2] * axis[2];

    Vec3 radial_direction{};

    for (std::size_t i = 0; i < 3; ++i) {
        radial_direction[i] = normal[i] - axial_component * axis[i];
    }

    const double radial_norm =
        std::hypot(radial_direction[0], radial_direction[1], radial_direction[2]);

    // 3. 从轮心出发，沿径向和轴向找到圆柱最低点。
    // 沿法向的反方向找最低点：圆周提供半径偏移，倾斜时端面边缘还贡献半轮宽。
    // 轮轴垂直法向时，轴向最低点不唯一，这里选宽度中间的代表点。
    Vec3 contact = wheel_center;

    for (std::size_t i = 0; i < 3; ++i) {
        if (radial_norm > EPSILON) {
            contact[i] -= radius_m * radial_direction[i] / radial_norm;
        }

        if (axial_component > EPSILON) {
            contact[i] -= 0.5 * width_m * axis[i];
        } else if (axial_component < -EPSILON) {
            contact[i] += 0.5 * width_m * axis[i];
        }
    }

    return contact;
}

std::array<Vec3, LEG_COUNT> estimateWheelContactPoints(const FourLegKinematics& kinematics,
                                                       const SensorSample& feedback) {
    if (!std::isfinite(feedback.roll_rad) || !std::isfinite(feedback.pitch_rad)) {
        throw std::invalid_argument("Body attitude must be finite");
    }

    // 平地法向：由世界坐标系转换到机身坐标系。
    // 这是把世界 +Z 方向逆旋转到 base；绕世界竖直轴的 yaw 不改变该方向。
    // 坡地或未知地面需要另行估计法向，不能直接继续用水平地面假设。
    const Vec3 ground_normal_base{-std::sin(feedback.pitch_rad),
                                  std::cos(feedback.pitch_rad) * std::sin(feedback.roll_rad),
                                  std::cos(feedback.pitch_rad) * std::cos(feedback.roll_rad)};

    const WheelCenters centers = kinematics.forward(feedback.joint_angles);

    std::array<Vec3, LEG_COUNT> contacts{};

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        // 轮子局部 Y 轴上的辅助点。
        // 先变换一个轮轴上的点，再减去轮心抵消平移，得到方向；方向后续会归一化。
        const Vec3 axis_point = kinematics.pointToBase(
            leg_index, LegLink::Wheel, Vec3{0.0, 1.0, 0.0}, feedback.joint_angles, feedback.angles);

        // 减去轮心，得到轮轴方向。
        Vec3 axis_base{};
        for (std::size_t i = 0; i < 3; ++i) {
            axis_base[i] = axis_point[i] - centers[leg_index][i];
        }

        contacts[leg_index] = estimateWheelContactPoint(
            centers[leg_index], axis_base, ground_normal_base, WHEEL_RADIUS_M, WHEEL_WIDTH_M);
    }

    return contacts;
}

}  // namespace wl
