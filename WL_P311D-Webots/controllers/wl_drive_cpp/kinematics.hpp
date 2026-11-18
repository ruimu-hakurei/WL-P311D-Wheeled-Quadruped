// 【运动学接口】关节角与轮心/连杆局部点之间的几何转换。
// 只保存几何参数；不读取传感器、不推进时间、不发送电机目标。
#pragma once

#include "models.hpp"

#include <vector>

namespace wl {

// 纯数学模块：X-Y-Y 串联关节，轮心位置在机身坐标系中，单位 m、rad。
class LegKinematics {
public:
    explicit LegKinematics(LegGeometry geometry);

    // 同名重载：既可以传三个角，也可以传 JointAngles；最终复用同一份正解实现。
    // 函数结尾的 const 表示计算不会修改本对象保存的几何参数。
    Vec3 forward(double haa, double hfe, double kfe) const;
    Vec3 forward(const JointAngles& angles) const;

    // 候选解尚未检查电机限位；inverse() 再筛选并按参考姿态选解。
    std::vector<JointAngles> inverseCandidates(const Vec3& target) const;
    JointAngles inverse(const Vec3& target, const JointLimits& limits,
                        const JointAngles& reference = JointAngles{}) const;
    // 将某个连杆局部坐标系中的点转换到机身坐标系。
    Vec3 pointToBase(LegLink link, const Vec3& local_point, const JointAngles& angles,
                     double wheel_angle_rad = 0.0) const;

private:
    LegGeometry geometry_;
};

class FourLegKinematics {
public:
    explicit FourLegKinematics(const std::array<LegGeometry, LEG_COUNT>& geometries);

    // 四腿关节角 → 四个轮心位置。
    WheelCenters forward(const LegPoses& angles) const;

    // 四个轮心目标 → 四腿关节目标。
    // reference 用于选择接近当前姿态的逆解分支。
    LegPoses inverse(const WheelCenters& targets, const LegJointLimits& limits,
                     const LegPoses& reference) const;
    // 按腿编号选择对应运动学对象及实测角度。
    Vec3 pointToBase(std::size_t leg_index, LegLink link, const Vec3& local_point,
                     const LegPoses& angles, const WheelValues& wheel_angles) const;

private:
    std::array<LegKinematics, LEG_COUNT> legs_;
};

}  // namespace wl
