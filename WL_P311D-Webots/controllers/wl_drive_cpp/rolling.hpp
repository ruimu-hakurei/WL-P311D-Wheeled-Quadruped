// 【轮式控制接口】键盘指令 → 航向修正 → 左右轮速分配。
// forward / turn 都是轮速分配量，单位 rad/s；不是机身线速度和角速度。
#pragma once

#include "models.hpp"

#include <optional>
#include <set>

namespace wl {

// 当前仅为带限幅的航向 P 控制器，输出仍是轮速差修正量。
class HeadingController {
public:
    explicit HeadingController(double kp = 1.0, double output_limit = 0.5);
    void reset(std::optional<double> yaw);
    double update(double yaw);

    double error() const { return error_; }

private:
    double kp_;
    double output_limit_;
    double error_ = 0.0;
    std::optional<double> target_yaw_;
};

// 轮子保持的状态变化，供入口层打印原因；不在算法层操作电机。
enum class WheelHoldEvent {
    None,
    Enabled,
    Released,
    FeedbackLost,
    RejectedStand,
    RejectedMotion,
    RejectedFeedback
};

struct WheelHoldGoal {
    std::optional<WheelValues> target_angles;
    WheelHoldEvent event = WheelHoldEvent::None;
};

// 站姿下的 H/R 验证状态机：只保存一次进入时的轮子角度。
// 期望保持状态在这里；上次实际发送记录仍由 RobotIO 保存，二者职责不同。
class WheelHoldController {
public:
    explicit WheelHoldController(double speed_tolerance_rad_s);

    WheelHoldGoal update(const std::set<int>& keys, const std::optional<SensorSample>& feedback,
                         bool stand_reached, const MotionCommand& requested_motion);

private:
    double speed_tolerance_rad_s_;
    bool hold_key_was_down_ = false;
    std::optional<WheelValues> target_angles_;
};

// 都是纯计算函数，可以在不启动 Webots 的情况下核对结果。
WheelCommand differentialWheelCommand(double forward, double turn, double max_speed);
MotionCommand decodeKeyboardCommand(const std::set<int>& keys);

}  // namespace wl
