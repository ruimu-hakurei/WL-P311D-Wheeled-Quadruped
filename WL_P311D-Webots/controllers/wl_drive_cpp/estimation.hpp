// 【模型估计接口】质量加权质心、圆柱轮几何接地点。
// 本文件没有实现 IMU/GPS 融合、滤波、滑移估计或真实接触检测。
#pragma once

#include "models.hpp"

#include <vector>

#include "kinematics.hpp"

namespace wl {

// 所有部件的位置必须转到同一个机身坐标系
Vec3 computeCenterOfMass(const std::vector<MassPoint>& points);

// 根据模型参数和实测关节角，计算机身坐标系中的整体质心
Vec3 estimateCenterOfMass(const FourLegKinematics& kinematics, const LegPoses& joint_angles,
                          const WheelValues& wheel_angles);

// 三个向量必须表达在同一个坐标系中。
// ground_normal 指向地面上方；返回圆柱轮子的几何候选接触点。
Vec3 estimateWheelContactPoint(const Vec3& wheel_center, const Vec3& wheel_axis,
                               const Vec3& ground_normal, double radius_m, double width_m);

// 四轮版本假设世界地面水平，由 roll/pitch 求地面法向在 base 中的表达。
// 输出仍在 base 中，不附带真实接地标志或接触力。
std::array<Vec3, LEG_COUNT> estimateWheelContactPoints(const FourLegKinematics& kinematics,
                                                       const SensorSample& feedback);

}  // namespace wl
