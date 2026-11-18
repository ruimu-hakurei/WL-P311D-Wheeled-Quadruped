#pragma once

// 启动参数与隔离验证辅助功能。第一次学控制链路时可以跳过此文件。
#include <cstddef>
#include <filesystem>
#include <set>
#include <array>
#include <optional>
#include <string>
#include "models.hpp"

namespace wl {

struct RunOptions {
    double validation_seconds = 0.0;  // 0 表示正常运行，不按计时自动结束。
    bool scripted_input = false;
    bool scripted_trot = false;
    bool quit_webots = false;
};

RunOptions parseRunOptions(int argc, char** argv);
std::set<int> scriptedKeysForValidation(double elapsed_s);
std::set<int> scriptedKeysForTrotValidation(double elapsed_s);
bool shouldFinishValidation(const RunOptions& options, double elapsed_s);

// 每段运动独立积累实测前移和偏航。只在小跑脚本测试时采样。
struct MotionMeasurement {
    std::optional<Vec3> previous;  // 世界系 x、y 和偏航角
    double forward_m = 0.0;
    double yaw_rad = 0.0;
    void observe(const SensorSample& sample);
};

struct TrotValidation {
    MotionMeasurement straight, left, right, arc, rolling;
    std::size_t landed_steps = 0;
    std::string first_fault;
    bool exited = false;
    bool finished_input = false;
    void observe(const SensorSample& sample, const std::set<int>& keys, bool trotting,
                 bool wheels_in_velocity_mode, double elapsed_s);
    bool passed(std::size_t invalid_frames) const;
};

void writeValidationCompletion(const std::filesystem::path& controller_folder,
                               std::size_t valid_frames, std::size_t invalid_frames,
                               double last_time_s, const TrotValidation* trot);

}  // namespace wl
