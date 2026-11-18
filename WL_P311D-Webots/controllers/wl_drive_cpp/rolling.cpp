// 【轮式算法】纯计算，不调用 Motor；最终执行仍交给 RobotIO。
#include "rolling.hpp"
#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wl {
namespace {

// 保持纯计算模块不依赖 Webots；入口对 SDK 的真实编号做 static_assert。
constexpr int LEFT_ARROW = 314;
constexpr int UP_ARROW = 315;
constexpr int RIGHT_ARROW = 316;
constexpr int DOWN_ARROW = 317;

bool isPressed(const std::set<int>& keys, int letter, int arrow) {
    return keys.count(letter) != 0 || keys.count(arrow) != 0;
}

}  // namespace

HeadingController::HeadingController(double kp, double output_limit)
    : kp_(kp), output_limit_(output_limit) {
    if (!std::isfinite(kp_) || !std::isfinite(output_limit_) || output_limit_ < 0.0) {
        throw std::invalid_argument("Invalid heading controller gains");
    }
}

void HeadingController::reset(std::optional<double> yaw) {
    target_yaw_ = yaw;
    error_ = 0.0;
}

double HeadingController::update(double yaw) {
    if (!target_yaw_.has_value()) {
        reset(yaw);
        return 0.0;
    }

    // 航向误差取最短方向，处理跨越正负 pi 的情况。
    const double difference = target_yaw_.value() - yaw;
    error_ = std::atan2(std::sin(difference), std::cos(difference));
    const double correction = kp_ * error_;
    return std::clamp(correction, -output_limit_, output_limit_);
}

WheelCommand differentialWheelCommand(double forward, double turn, double max_speed) {
    if (!std::isfinite(forward) || !std::isfinite(turn) || !std::isfinite(max_speed)
        || max_speed <= 0.0) {
        throw std::invalid_argument("Wheel command and speed limit must be finite");
    }

    // 正转向量让左轮减速、右轮加速；同侧前后轮由 RobotIO 分配同一目标。
    const double desired_left = forward - turn;
    const double desired_right = forward + turn;

    // 同比缩放，保留左右轮的速度比例。
    const double scale =
        std::max({1.0, std::abs(desired_left) / max_speed, std::abs(desired_right) / max_speed});
    return WheelCommand{desired_left / scale, desired_right / scale};
}

MotionCommand decodeKeyboardCommand(const std::set<int>& keys) {
    if (keys.count(' ') != 0) {
        return {};  // 空格优先停车，即使同时按着 W/A 等键。
    }

    // bool 转 int 得到 0/1，相反方向同时按下时相减抵消；空格已在前面优先处理。
    const int forward_direction =
        int(isPressed(keys, 'W', UP_ARROW)) - int(isPressed(keys, 'S', DOWN_ARROW));
    const int turn_direction =
        int(isPressed(keys, 'A', LEFT_ARROW)) - int(isPressed(keys, 'D', RIGHT_ARROW));

    MotionCommand command;
    command.forward = FORWARD_SPEED * forward_direction;
    command.turn = TURN_SPEED * turn_direction;
    return command;
}

WheelHoldController::WheelHoldController(double speed_tolerance_rad_s)
    : speed_tolerance_rad_s_(speed_tolerance_rad_s) {
    if (!std::isfinite(speed_tolerance_rad_s_) || speed_tolerance_rad_s_ <= 0.0) {
        throw std::invalid_argument("Wheel hold entry speed tolerance must be finite and positive");
    }
}

WheelHoldGoal WheelHoldController::update(const std::set<int>& keys,
                                          const std::optional<SensorSample>& feedback,
                                          bool stand_reached,
                                          const MotionCommand& requested_motion) {
    // 识别 H 从未按下到按下的变化；拒绝后需要松开再按，不会在条件变好时自动进入。
    const bool hold_key_down = keys.count('H') != 0;
    const bool request_hold = hold_key_down && !hold_key_was_down_;
    hold_key_was_down_ = hold_key_down;

    if (!feedback.has_value()) {
        const bool was_holding = target_angles_.has_value();
        target_angles_.reset();
        const WheelHoldEvent event =
            was_holding ? WheelHoldEvent::FeedbackLost
                        : (request_hold ? WheelHoldEvent::RejectedFeedback : WheelHoldEvent::None);
        return {target_angles_, event};
    }

    // R 优先于 H；退出之后不保留旧角度，下次 H 重新捕获。
    if (keys.count('R') != 0) {
        const bool was_holding = target_angles_.has_value();
        target_angles_.reset();
        return {target_angles_, was_holding ? WheelHoldEvent::Released : WheelHoldEvent::None};
    }

    // 保持期间不重新读取并替换目标；即使 H 自动重复，也始终使用首次捕获的角度。
    // 站姿阶段偶尔回到 WaitReached 时继续保持，只有进入时要求 Stand=HOLD。
    if (target_angles_.has_value() || !request_hold) {
        return {target_angles_, WheelHoldEvent::None};
    }

    if (!stand_reached) {
        return {target_angles_, WheelHoldEvent::RejectedStand};
    }

    if (!feedback->speeds_valid || requested_motion.forward != 0.0
        || requested_motion.turn != 0.0) {
        return {target_angles_, WheelHoldEvent::RejectedMotion};
    }

    // 检查整组测量后再保存，避免只采集了部分轮子。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        const double angle = feedback->angles[leg_index];
        const double speed = feedback->speeds[leg_index];
        if (!std::isfinite(angle) || !std::isfinite(speed)) {
            return {target_angles_, WheelHoldEvent::RejectedFeedback};
        }
        if (std::abs(speed) > speed_tolerance_rad_s_) {
            return {target_angles_, WheelHoldEvent::RejectedMotion};
        }
    }

    // 值拷贝冻结进入瞬间的四个连续角度；这里不调用任何 SDK 电机函数。
    target_angles_ = feedback->angles;
    return {target_angles_, WheelHoldEvent::Enabled};
}

}  // namespace wl
