// 【运行快照】启动时保存配置、可用力矩和磁盘文件指纹，不逐拍执行。
// 快照证明所列文件的磁盘内容，不证明运行中的可执行文件与之完全一致。
// 启动时的参数与文件指纹快照。逐拍 CSV 记录实现在 recorder.cpp。
#include "recorder.hpp"
#include "config.hpp"
#include "file_snapshot.hpp"
#include "robot_io.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace wl {
namespace {

using FileEntry = std::pair<std::string, std::filesystem::path>;

std::string finiteNumber(double value) {
    if (!std::isfinite(value)) {
        throw std::runtime_error("Non-finite number in metadata");
    }
    std::ostringstream text;
    text << std::setprecision(17) << value;
    return text.str();
}

std::vector<FileEntry> snapshotFiles(const std::filesystem::path& controller_folder,
                                     const std::filesystem::path& world_path) {
    std::vector<FileEntry> files;
    // 启动时记录控制源码；新加模块应同步加入清单，文件指纹只描述磁盘内容。
    for (const auto* relative_path :
         {"wl_drive_cpp.cpp",  "config.hpp",        "models.hpp",        "robot_io.hpp",
          "robot_io.cpp",      "rolling.hpp",       "rolling.cpp",       "kinematics.hpp",
          "kinematics.cpp",    "trajectory.hpp",    "trajectory.cpp",    "motion.hpp",
          "motion.cpp",        "body_control.hpp",  "body_control.cpp",  "recorder.hpp",
          "recorder.cpp",      "metadata.cpp",      "file_snapshot.hpp", "file_snapshot.cpp",
          "detail/sha256.hpp", "detail/sha256.cpp", "run_options.hpp",   "run_options.cpp",
          "gait.hpp",          "gait.cpp",          "estimation.hpp",    "estimation.cpp",
          "crawl_controller.hpp", "crawl_controller.cpp", "trot_controller.hpp", "trot_controller.cpp",
          "wheel_ground.hpp", "wheel_ground.cpp",
          "Makefile",          ".clang-format"}) {
        files.emplace_back(relative_path, controller_folder / relative_path);
    }

    const auto project_folder = controller_folder.parent_path().parent_path();
    files.emplace_back("world", world_path);
    files.emplace_back("proto", project_folder / "protos/WL_P311D.proto");
    return files;
}

void writeConfiguration(std::ostream& output, int time_step_ms, bool validation_script) {
    output
        << std::setprecision(17) << "{\n  \"schema_version\": 5,"
        << "\n  \"implementation_language\": \"C++17\","
        << "\n  \"time_step_ms\": " << time_step_ms
        << ",\n  \"forward_speed_rad_s\": " << FORWARD_SPEED
        << ",\n  \"turn_speed_rad_s\": " << TURN_SPEED
        << ",\n  \"wheel_acceleration_rad_s2\": " << WHEEL_ACCELERATION
        << ",\n  \"wheel_hold_max_speed_rad_s\": " << WHEEL_HOLD_MAX_SPEED_RAD_S
        << ",\n  \"wheel_hold_entry_speed_tolerance_rad_s\": "
        << WHEEL_HOLD_ENTRY_SPEED_TOLERANCE_RAD_S << ",\n  \"wheel_hold_keyboard_enabled\": true"
        << ",\n  \"csv_column_count\": " << ExperimentRecorder::fields().size()
        << ",\n  \"wheel_output_timing\": "
        << jsonString("Mode and targets describe the preceding execution interval")
        << ",\n  \"inactive_wheel_target_columns_are_empty\": true"
        << ",\n  \"settle_seconds\": " << SETTLE_SECONDS
        << ",\n  \"joint_error_definition\": \"target minus measured joint angle, radians\","
        << "\n  \"reference_center_offset_base_m\": [" << REFERENCE_CENTER_X_M << ",0,0],"
        << "\n  \"reference_center_note\": "
        << jsonString(
               "Body-fixed zero-pose wheel geometry reference; not COM or instantaneous rotation center")
        << ",\n  \"file_snapshots_are_on_disk_not_live_world_state\": true,"
        << "\n  \"leg_trajectory_is_preview_only\": false,"
        << "\n  \"stand_transition_is_implemented_not_connected\": false,"
        << "\n  \"validation_script\": " << (validation_script ? "true" : "false");
}

void writeAvailableTorques(std::ostream& output, const RobotIO& robot_io) {
    const LegPoses available_torques = robot_io.availableJointTorques();
    output << ",\n  \"joint_available_torque_Nm\": {";
    bool first_entry = true;
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            if (!first_entry) {
                output << ',';
            }
            first_entry = false;
            const std::string name = std::string(LEGS[leg_index]) + "_" + LEG_JOINTS[joint_index];
            output << "\n    " << jsonString(name) << ": "
                   << finiteNumber(available_torques[leg_index][joint_index]);
        }
    }
    output << "\n  }";
}

void writeFileSnapshots(std::ostream& output, const std::vector<FileEntry>& files) {
    output << ",\n  \"files\": {";
    bool first_entry = true;
    for (const auto& entry : files) {
        if (!first_entry) {
            output << ',';
        }
        first_entry = false;
        output << "\n    " << jsonString(entry.first)
               << ": {\"path\": " << jsonString(entry.second.u8string()) << ", \"sha256\": ";

        // 文件不可读时留下原因，不因一份源文件的权限问题中断控制。
        try {
            output << jsonString(sha256File(entry.second));
        } catch (const std::exception& error) {
            output << "null, \"error\": " << jsonString(error.what());
        }
        output << '}';
    }
    output << "\n  }";
}

}  // namespace

std::string runMetadata(const RobotIO& robot_io, int time_step_ms,
                        const std::filesystem::path& controller_folder,
                        const std::filesystem::path& world_path, bool validation_script) {
    std::ostringstream metadata;
    writeConfiguration(metadata, time_step_ms, validation_script);
    writeAvailableTorques(metadata, robot_io);
    writeFileSnapshots(metadata, snapshotFiles(controller_folder, world_path));
    metadata << "\n}";
    return metadata.str();
}

}  // namespace wl
