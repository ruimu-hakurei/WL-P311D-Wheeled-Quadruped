#pragma once

#include "body_control.hpp"
#include "wheel_ground.hpp"

#include <array>
#include <optional>
#include <string>

namespace webots {
class Node;
class Supervisor;
}

namespace wl {

// 对角小跑控制器。跨帧保存换步状态，生成腿关节和轮角目标，交给 RobotIO 下发。
class TrotController {
public:
    TrotController(webots::Supervisor& robot, const FourLegKinematics& kinematics,
                   bool log_contacts);
    // 每个仿真步调用一次。feedback 用 const& 借用本帧数据，返回本帧执行目标的值。
    // forward_m_s < 0 请求停步，等摆动轮落地。yaw_step_rad 是每次换步的偏航输入。
    ActuatorTargets update(const SensorSample& feedback, double forward_m_s, double yaw_step_rad,
                           bool exit_requested);
    void onFeedbackLost();
    std::size_t landedSteps() const { return count_; }
    const std::string& firstFault() const { return first_fault_; }
    bool failed() const { return phase_ == Phase::Failed; }
    bool standing() const { return phase_ == Phase::Idle; }
    bool readyToExit() const { return ready_to_exit_; }

private:
    // enum class 将枚举值限制在所属类型，例如 Phase::Swing。
    // Phase 表示本对腿进行到哪一步，Motion 表示这一对腿按什么方向迈步。
    enum class Phase { Idle, Transfer, Swing, Landing, Recenter, Failed };
    enum class Motion { Forward, PivotLeft, PivotRight, ArcLeft, ArcRight };
    using Contacts = std::array<bool, LEG_COUNT>;

    // 走法调度与状态更新。返回 true 表示已开始下一步或已确认落地。
    bool beginNextStep(const SensorSample& feedback, double forward_m_s, double yaw_step_rad,
                       bool grounded);
    void updateRecenter(const SensorSample& feedback, double elapsed_s, bool grounded,
                        bool exit_requested);
    bool updateSwingAndLanding(const SensorSample& feedback, const Contacts& contacts,
                               double tilt_rad, double elapsed_s, bool grounded, int contact_frames);

    // 阶段入口冻结这一段的起始反馈，采样函数按经过的时间生成位置目标。
    void beginRecenter(const SensorSample& feedback);
    ActuatorTargets sampleRecenter(const SensorSample& feedback) const;
    void beginTransfer(const SensorSample& feedback, double forward_m_s, double yaw_step_rad);
    void beginSwing(const SensorSample& feedback);
    Vec3 swingEnd(std::size_t leg) const;
    WheelCenters sampleTransferCenters(double elapsed_s) const;
    WheelCenters sampleSwingCenters(const SensorSample& feedback, double elapsed_s) const;
    ActuatorTargets sampleTargets(const SensorSample& feedback) const;
    bool needsWorkspaceRecenter(const SensorSample& feedback) const;
    bool swings(std::size_t leg) const;

    // 运动学对象由主循环持有，Webots 节点指针归仿真器管理。
    const FourLegKinematics& kinematics_;
    const WheelCenters nominal_centers_;
    const bool log_contacts_;
    webots::Node* body_;
    const double floor_z_m_;
    WheelNodes wheels_{};
    void fail(const std::string& reason);
    std::string first_fault_;
    Phase phase_ = Phase::Idle;
    bool ready_to_exit_ = false;
    // 起相位冻结输入快照和轮心，轨迹用经过的时间采样。
    SensorSample phase_start_{};
    WheelCenters references_{};
    Vec3 body_shift_{};
    Vec3 swing_body_advance_{};
    double step_m_ = 0.0;
    double step_yaw_rad_ = 0.0;
    double step_lateral_m_ = 0.0;
    bool turning_ = false;
    // 原地转保存起转位置，直行保存起始航向和位置。optional 为空表示尚未建立参考。
    std::optional<std::array<double, 2>> turn_origin_m_;
    Vec3 turn_center_correction_{};
    std::optional<double> forward_heading_rad_;
    std::array<double, 2> forward_origin_m_{};
    std::size_t diagonal_ = 0;
    std::size_t count_ = 0;
    std::size_t moving_transfers_ = 0;
    std::optional<Motion> previous_motion_;
    std::optional<StandTransition> recenter_;
    // 计数归零后才开始确认下一个阶段。离地掩码保存本步各轮是否曾经离地。
    int grounded_frames_ = 0;
    std::array<bool, LEG_COUNT> saw_airborne_{};
    std::optional<ActuatorTargets> last_targets_;  // 首帧初始化，保护停步时复用。
};

}  // namespace wl
