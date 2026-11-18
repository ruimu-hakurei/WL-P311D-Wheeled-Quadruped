// 手动触发的一轮爬行。CrawlGait 管腿序，本类协调支撑、落地确认和回中。
#pragma once

#include "body_control.hpp"
#include "wheel_ground.hpp"

#include <optional>

namespace webots {
class Node;
class Supervisor;
}

namespace wl {

class CrawlController {
public:
    explicit CrawlController(webots::Supervisor& robot);
    void start(const FourLegKinematics& kinematics, const SensorSample& feedback,
               const MotionCommand& command);
    void requestStop();
    ActuatorTargets update(const FourLegKinematics& kinematics, const SensorSample& feedback);

    bool running() const;
    bool cycleCompleted() const;
    void hold(const char* reason);
    void clearPausedStep();

private:
    using Contacts = std::array<bool, LEG_COUNT>;

    Vec3 wheelOffsetWorld() const;
    Vec3 recenterErrorWorld() const;
    void beginSupportShift(const FourLegKinematics& kinematics, const SensorSample& feedback);
    void beginRecenter(const FourLegKinematics& kinematics, const SensorSample& feedback);
    ActuatorTargets shiftTargets(const FourLegKinematics& kinematics, double elapsed_s) const;
    ActuatorTargets swingTargets(const FourLegKinematics& kinematics, double progress) const;

    webots::Node* body_;
    const double floor_z_m_;
    WheelNodes wheels_{};
    CrawlGait gait_;
    GaitGoal goal_{};
    CrawlStepPlan step_{};
    SensorSample start_{};
    std::optional<ActuatorTargets> last_targets_;
    Vec3 initial_wheel_offset_{};
    WheelCenters turn_end_world_{};
    double step_length_m_ = 0.0;
    double target_yaw_rad_ = 0.0;
    double recenter_yaw_rad_ = 0.0;
    double wait_started_s_ = 0.0;
    int support_samples_ = 0;
    int landing_samples_ = 0;
    std::size_t completed_steps_ = 0;
    bool turning_ = false;
    bool forward_after_turn_ = false;
    bool saw_airborne_ = false;
    bool recentering_ = false;
    bool stop_requested_ = false;
    bool running_ = false;
    bool cycle_completed_ = false;
    bool paused_after_landing_ = false;
    MotionCommand cycle_command_{};
};

}  // namespace wl
