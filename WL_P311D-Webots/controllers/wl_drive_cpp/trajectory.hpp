// 【轨迹接口】负责“起终点之间怎样平滑变化”，不决定哪条腿先动。
// 持续保存起点/开始时刻的对象应创建一次，逐拍调用 sample。
#pragma once

#include "models.hpp"

namespace wl {

// 返回 [0,1] 中的无量纲进度；时间单位 s。
double quinticProgress(double elapsed_s, double duration_s);

// 起终点必须属于同一个坐标系。
// progress 为 0～1 的摆动进度，lift_height_m 为沿 Z 轴的抬高量。
Vec3 swingPosition(const Vec3& start, const Vec3& end, double progress, double lift_height_m);

// 起终点属于同一坐标系，单位 m。
Vec3 interpolatePosition(const Vec3& start, const Vec3& end, double elapsed_s, double duration_s);

// 构造时保存一次起始角度，随后按当前仿真时间计算目标。
class JointTransition {
public:
    JointTransition(JointAngles start_angles, JointAngles end_angles, double start_time_s,
                    double duration_s);

    JointAngles sample(double now_s) const;

private:
    JointAngles start_angles_;
    JointAngles end_angles_;
    double start_time_s_;
    double duration_s_;
};

}  // namespace wl
