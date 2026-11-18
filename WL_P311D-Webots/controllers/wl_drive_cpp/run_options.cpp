// 【启动辅助实现】解析显式验证选项；正常交互控制不会自动执行脚本按键。
#include "run_options.hpp"
#include "file_snapshot.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace wl {
namespace {
constexpr double SCRIPT_END_S = 80.0;
}  // namespace

RunOptions parseRunOptions(int argc, char** argv) {
    RunOptions options;
    const std::string duration_prefix = "--validation-seconds=";

    // 从 1 开始跳过程序名；只解析显式选项，默认 options 保持正常交互运行。
    for (int argument_index = 1; argument_index < argc; ++argument_index) {
        const std::string argument = argv[argument_index];
        if (argument.empty()) {
            continue;
        }

        if (argument.rfind(duration_prefix, 0) == 0) {
            const std::string duration_text = argument.substr(duration_prefix.size());
            std::size_t consumed_characters = 0;
            options.validation_seconds = std::stod(duration_text, &consumed_characters);

            const bool parsed_all_characters = consumed_characters == duration_text.size();
            const bool valid_duration =
                std::isfinite(options.validation_seconds) && options.validation_seconds > 0.0;
            if (!parsed_all_characters || !valid_duration) {
                throw std::invalid_argument("Invalid validation duration");
            }
        } else if (argument == "--scripted-input") {
            options.scripted_input = true;
        } else if (argument == "--scripted-trot") {
            options.scripted_trot = true;
        } else if (argument == "--quit-webots") {
            options.quit_webots = true;
        } else {
            throw std::invalid_argument("Unknown controller argument: " + argument);
        }
    }

    if (options.scripted_input && options.scripted_trot) {
        throw std::invalid_argument("Select one validation input script");
    }
    const bool requested_validation_behavior =
        options.scripted_input || options.scripted_trot || options.quit_webots;
    if (requested_validation_behavior && options.validation_seconds <= 0.0) {
        throw std::invalid_argument("Validation options require --validation-seconds");
    }
    return options;
}

std::set<int> scriptedKeysForValidation(double elapsed_s) {
    // 和迁移验证时相同的时序；只在显式启用 --scripted-input 时调用。
    if (elapsed_s < 2.0) {
        return {};
    }
    if (elapsed_s < 3.0) {
        return {'W'};
    }
    if (elapsed_s < 4.0) {
        return {'W', 'A'};
    }
    if (elapsed_s < 5.0) {
        return {'A'};
    }
    if (elapsed_s < 6.0) {
        return {};
    }
    if (elapsed_s < 7.0) {
        return {'S'};
    }
    if (elapsed_s < 8.0) {
        return {'W', ' '};
    }
    return {};
}

std::set<int> scriptedKeysForTrotValidation(double elapsed_s) {
    struct InputInterval {
        double start_s;
        double end_s;
        std::set<int> keys;
    };
    static const std::array<InputInterval, 9> input{{
        {7.0, 7.3, {'H'}},
        {8.0, 8.3, {'T'}},
        {9.0, 30.0, {'W'}},
        {32.0, 46.0, {'A'}},
        {48.0, 62.0, {'D'}},
        {64.0, 71.0, {'W', 'A'}},
        {71.0, 72.0, {'W', ' '}},
        {73.0, 73.3, {'R'}},
        {78.0, SCRIPT_END_S, {'W'}},
    }};
    for (const auto& interval : input) {
        if (elapsed_s >= interval.start_s && elapsed_s < interval.end_s) {
            return interval.keys;
        }
    }
    return {};
}

bool shouldFinishValidation(const RunOptions& options, double elapsed_s) {
    return options.validation_seconds > 0.0 && elapsed_s >= options.validation_seconds;
}

void MotionMeasurement::observe(const SensorSample& sample) {
    const Vec3 pose{sample.reference_center_x_m, sample.reference_center_y_m, sample.yaw_rad};
    if (previous) {
        const double heading = (*previous)[2];
        const double turn = pose[2] - heading;
        // 投影到上一拍机头方向，避免把侧滑算作前进。
        forward_m += (pose[0] - (*previous)[0]) * std::cos(heading)
                   + (pose[1] - (*previous)[1]) * std::sin(heading);
        yaw_rad += std::atan2(std::sin(turn), std::cos(turn));
    }
    previous = pose;
}

void TrotValidation::observe(const SensorSample& sample, const std::set<int>& keys,
                             bool trotting, bool wheels_in_velocity_mode, double elapsed_s) {
    finished_input = elapsed_s >= SCRIPT_END_S;
    if (keys.count(' ')) return;
    if (trotting) {
        if (keys.count('W') && keys.count('A')) arc.observe(sample);
        else if (keys.count('W')) straight.observe(sample);
        else if (keys.count('A')) left.observe(sample);
        else if (keys.count('D')) right.observe(sample);
    } else if (exited && wheels_in_velocity_mode && keys.count('W')) {
        rolling.observe(sample);
    }
}

bool TrotValidation::passed(std::size_t invalid_frames) const {
    // 留足物理引擎差异的余量，仅验证脚本中的每种动作确实发生且方向正确。
    return finished_input && exited && invalid_frames == 0 && first_fault.empty()
           && landed_steps >= 2 && straight.forward_m > 0.05 && left.yaw_rad > 0.05
           && right.yaw_rad < -0.05 && arc.forward_m > 0.02 && arc.yaw_rad > 0.05
           && rolling.forward_m > 0.02;
}

void writeValidationCompletion(const std::filesystem::path& controller_folder,
                               std::size_t valid_frames, std::size_t invalid_frames,
                               double last_time_s, const TrotValidation* trot) {
    // simulationQuit 可能先结束控制台转发，磁盘标记用于确认校验完整结束。
    const auto marker_path = controller_folder / "logs/validation_completed.json";
    std::ofstream marker;
    marker.exceptions(std::ios::failbit | std::ios::badbit);
    marker.open(marker_path, std::ios::binary);
    marker << "{\"valid_frames\":" << valid_frames << ",\"invalid_frames\":" << invalid_frames
           << ",\"last_time_s\":" << last_time_s << ",\"completed\":true";
    if (trot) {
        marker << std::boolalpha << ",\"passed\":" << trot->passed(invalid_frames)
               << ",\"landed_steps\":" << trot->landed_steps
               << ",\"first_fault\":" << jsonString(trot->first_fault)
               << ",\"returned_to_rolling\":" << trot->exited
               << ",\"straight_forward_m\":" << trot->straight.forward_m
               << ",\"left_yaw_rad\":" << trot->left.yaw_rad
               << ",\"right_yaw_rad\":" << trot->right.yaw_rad
               << ",\"arc_forward_m\":" << trot->arc.forward_m
               << ",\"arc_yaw_rad\":" << trot->arc.yaw_rad
               << ",\"rolling_forward_m\":" << trot->rolling.forward_m;
    }
    marker << '}';
    marker.close();
}

}  // namespace wl
