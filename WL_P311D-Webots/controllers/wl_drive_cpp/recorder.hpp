// 【观测输出接口】保存反馈与上一执行区间已发送的轮子模式/目标，不参与决策。
#pragma once

#include "models.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace wl {

class RobotIO;

// 仅在启动时调用；实现位于 metadata.cpp，与逐拍 CSV 写入分开阅读。
std::string runMetadata(const RobotIO& robot_io, int time_step_ms,
                        const std::filesystem::path& controller_folder,
                        const std::filesystem::path& world_path, bool validation_script);

class ExperimentRecorder {
public:
    ExperimentRecorder(const std::filesystem::path& folder, const std::string& metadata);
    ~ExperimentRecorder();
    ExperimentRecorder(const ExperimentRecorder&) = delete;
    ExperimentRecorder& operator=(const ExperimentRecorder&) = delete;

    // 两份 previous 输出均来自 RobotIO 的上次发送记录，不使用本拍尚未发送的新目标。
    void record(const SensorSample& feedback, WheelCommand previous_command,
                const std::optional<WheelValues>& previous_hold_angles);
    void close();

    const std::filesystem::path& path() const { return path_; }

    const std::filesystem::path& metadataPath() const { return metadata_path_; }

    static std::vector<std::string> fields();

private:
    std::filesystem::path path_;
    std::filesystem::path metadata_path_;
    std::ofstream file_;
    double last_print_time_;
};

}  // namespace wl
