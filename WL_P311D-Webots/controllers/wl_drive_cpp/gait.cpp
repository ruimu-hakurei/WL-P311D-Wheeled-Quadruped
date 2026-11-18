// 【步态状态机】记住上拍状态，根据本拍条件转移，再生成本拍计划。
// 阶段结束、关节到位、实际接地是不同事件，不能用同一个计时替代。
#include "gait.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wl {

CrawlGait::CrawlGait(double swing_duration_s) : swing_duration_s_(swing_duration_s) {
    // 摆动时间必须是有限的正数。
    if (!std::isfinite(swing_duration_s_) || swing_duration_s_ <= 0.0) {
        throw std::invalid_argument("Swing duration must be finite and positive");
    }
}

GaitGoal CrawlGait::update(double now_s, bool support_ready, bool landing_confirmed) {
    if (!std::isfinite(now_s)) {
        throw std::invalid_argument("Gait update time must be finite");
    }

    // 摆动开始以后，仿真时间不能倒退。
    if (phase_ == GaitPhase::Swing && now_s < swing_start_time_s_) {
        throw std::invalid_argument("Gait time moved backwards");
    }

    // 1. 根据当前阶段，判断是否允许进入下一阶段。
    // 一个 update 只执行进入 switch 时选中的分支；break 退出 switch，不是退出整个函数。
    // 不在一拍里连跳多个阶段，后面的输出则按转移后的 phase_ 生成。
    switch (phase_) {
        case GaitPhase::ShiftSupport:
            // 支撑准备完成，才启动当前腿的摆动计时。
            if (support_ready) {
                swing_start_time_s_ = now_s;
                phase_ = GaitPhase::Swing;
            }
            break;

        case GaitPhase::Swing:
            // 轨迹时间结束，进入等待落地阶段。
            if (now_s - swing_start_time_s_ >= swing_duration_s_) {
                phase_ = GaitPhase::WaitContact;
            }
            break;

        case GaitPhase::WaitContact:
            // 当前腿确认落地后，选择下一条腿。
            if (landing_confirmed) {
                // 取模让索引按 0→1→2→3→0 循环，实际腿编号再查 leg_order_。
                order_index_ = (order_index_ + 1) % LEG_COUNT;
                phase_ = GaitPhase::ShiftSupport;
            }
            break;
    }

    // 2. 根据更新后的阶段，生成这一拍的步态计划。
    // 默认成员初始化先把四条腿都设为“计划接地”；只有 Swing 会把当前腿设为 false。
    // 计划接地不是传感器测到接地，WaitContact 仍必须等外部确认。
    GaitGoal goal;
    goal.phase = phase_;
    goal.active_leg = leg_order_[order_index_];

    if (phase_ == GaitPhase::Swing) {
        const double elapsed_s = now_s - swing_start_time_s_;

        goal.swing_progress = std::clamp(elapsed_s / swing_duration_s_, 0.0, 1.0);

        // 摆动腿计划离地，其余三条腿计划支撑。
        goal.planned_contact[goal.active_leg] = false;
    } else if (phase_ == GaitPhase::WaitContact) {
        // 轨迹保持终点，并计划接地；实际接地仍需确认。
        goal.swing_progress = 1.0;
    }

    return goal;
}

}  // namespace wl