// 【步态调度接口】只安排阶段、腿序和进度，不求逆解，也不操作电机。
// 当前尚未接入 runController；support_ready / landing_confirmed 仍需外层提供依据。
#pragma once

#include "models.hpp"

#include <array>
#include <cstddef>

namespace wl {
// 一条腿迈步时三个阶段
enum class GaitPhase {
    ShiftSupport,  // 移动机身/调整支撑，为抬腿做准备
    Swing,         // 摆动阶段
    WaitContact,   // 等待接触阶段

};

// 步态模块和每拍输出的计划
struct GaitGoal {
    GaitPhase phase = GaitPhase::ShiftSupport;  // 当前阶段

    // 当前准备抬起或等待落地的腿
    std::size_t active_leg = 0;

    // 当前摆动轨迹的进度，范围为0~1
    double swing_progress = 0.0;
    // 计划接地状态，顺序为 LF、RF、LH、RH。
    // 这是计划，实际是否接地需要另行确认。
    std::array<bool, LEG_COUNT> planned_contact = {true, true, true, true};
};

// 慢速四拍步态：安排阶段和腿的顺序，输出运动计划。
class CrawlGait {
public:
    explicit CrawlGait(double swing_duration_s);

    // support_ready：外层确认支撑已准备好；landing_confirmed：外层确认摆动腿落地。
    // 不能因为进入 WaitContact，就把 landing_confirmed 自动设为 true。
    GaitGoal update(double now_s, bool support_ready, bool landing_confirmed);

private:
    // 每次只摆动一条腿。
    // 保存的是腿编号，0/3/1/2 对应 LF/RH/RF/LH；order_index_ 则是序列中的位置。
    std::array<std::size_t, LEG_COUNT> leg_order_{0, 3, 1, 2};

    std::size_t order_index_ = 0;
    GaitPhase phase_ = GaitPhase::ShiftSupport;

    double swing_duration_s_;
    double swing_start_time_s_ = 0.0;
};
}  // namespace wl
