// 【日志实现】CSV 字段名与数据必须同序；控制算法内部使用 rad，摘要才换成 deg。
// schema 5 共 85 列；模型质心仍由主循环打印，没有加入 CSV。
#include "recorder.hpp"
#include "config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace wl {
namespace {

using CsvRow = std::vector<std::string>;

std::string number(double value) {
    if (!std::isfinite(value)) {
        throw std::runtime_error("Non-finite number in diagnostic output");
    }
    std::ostringstream text;
    text << std::setprecision(17) << value;
    return text.str();
}

void writeCsvRow(std::ostream& output, const CsvRow& row) {
    for (std::size_t column = 0; column < row.size(); ++column) {
        if (column != 0) {
            output << ',';
        }
        const std::string& value = row[column];
        const bool needs_quotes = value.find_first_of(",\"\r\n") != std::string::npos;
        if (!needs_quotes) {
            output << value;
            continue;
        }

        output << '"';
        for (char character : value) {
            if (character == '"') {
                output << '"';
            }
            output << character;
        }
        output << '"';
    }
    output << "\r\n";
}

std::string timestampForFilename() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    const auto microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()
        % 1000000;
    const std::tm local_time = *std::localtime(&seconds);

    std::ostringstream text;
    text << std::put_time(&local_time, "%Y%m%d_%H%M%S_") << std::setfill('0') << std::setw(6)
         << microseconds;
    return text.str();
}

void appendVector(CsvRow& row, const Vec3& vector) {
    for (double component : vector) {
        row.push_back(number(component));
    }
}

// 同一份 optional 决定模式标签和四个保持目标，避免两处状态彼此矛盾。
const char* wheelControlMode(const std::optional<WheelValues>& hold_angles) {
    return hold_angles.has_value() ? "POSITION_HOLD" : "VELOCITY";
}

CsvRow makeDataRow(const SensorSample& feedback, WheelCommand previous_command,
                   const std::optional<WheelValues>& previous_hold_angles) {
    // 原 80 列保持字段顺序，末尾追加模式与四个保持角度。
    // 位置保持时，轮速目标列不适用，留空；实测速度列仍正常记录。
    CsvRow row{number(feedback.time_s),
               number(feedback.roll_rad),
               number(feedback.pitch_rad),
               number(feedback.yaw_rad),
               number(feedback.horizontal_displacement_m),
               number(feedback.max_joint_error_rad),
               feedback.worst_joint,
               number(feedback.reference_center_x_m),
               number(feedback.reference_center_y_m),
               number(feedback.reference_center_displacement_m),
               previous_hold_angles.has_value() ? "" : number(previous_command.left),
               previous_hold_angles.has_value() ? "" : number(previous_command.right)};
    appendVector(row, feedback.position);
    appendVector(row, feedback.world_velocity);
    appendVector(row, feedback.angular_velocity);
    appendVector(row, feedback.lf_wheel_center_base_m);

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        row.push_back(number(feedback.angles[leg_index]));
        row.push_back(feedback.speeds_valid ? number(feedback.speeds[leg_index]) : "");
        for (std::size_t joint_index = 0; joint_index < JOINT_COUNT; ++joint_index) {
            row.push_back(number(feedback.joint_targets[leg_index][joint_index]));
            row.push_back(number(feedback.joint_angles[leg_index][joint_index]));
            row.push_back(number(feedback.joint_errors[leg_index][joint_index]));
            row.push_back(
                feedback.speeds_valid ? number(feedback.joint_speeds[leg_index][joint_index]) : "");
        }
    }
    row.push_back(wheelControlMode(previous_hold_angles));
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        row.push_back(previous_hold_angles.has_value()
                          ? number(previous_hold_angles.value()[leg_index])
                          : "");
    }
    return row;
}

void printFeedbackSummary(const SensorSample& feedback, WheelCommand previous_command,
                          const std::optional<WheelValues>& previous_hold_angles) {
    constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
    std::ostringstream text;
    text << std::fixed << std::setprecision(3) << "t=" << feedback.time_s << "s"
         << " yaw=" << feedback.yaw_rad * radians_to_degrees << "deg"
         << " roll=" << feedback.roll_rad * radians_to_degrees << "deg"
         << " pitch=" << feedback.pitch_rad * radians_to_degrees << "deg"
         << " reference_displacement=" << feedback.reference_center_displacement_m << "m"
         << " worst=" << feedback.worst_joint << ':'
         << feedback.max_joint_error_rad * radians_to_degrees << "deg"
         << " wheel_mode=" << wheelControlMode(previous_hold_angles);

    if (previous_hold_angles.has_value()) {
        // 与同一执行区间的编码器测量比较，显示最大轮轴位置误差。
        double max_hold_error_rad = 0.0;
        text << " hold_targets_rad=(";
        for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
            if (leg_index != 0) {
                text << ',';
            }
            const double target = previous_hold_angles.value()[leg_index];
            text << LEGS[leg_index] << ':' << target;
            max_hold_error_rad =
                std::max(max_hold_error_rad, std::abs(target - feedback.angles[leg_index]));
        }
        text << ") max_hold_error_rad=" << max_hold_error_rad;
    } else {
        text << " left/right=" << previous_command.left << '/' << previous_command.right;
    }
    std::cout << text.str() << std::endl;
}

}  // namespace

std::vector<std::string> ExperimentRecorder::fields() {
    std::vector<std::string> names{
        // 时间、姿态、位移、关节误差与实际下发的左右轮速。
        "time_s", "roll_rad", "pitch_rad", "yaw_rad", "horizontal_displacement_m",
        "max_joint_error_rad", "worst_joint", "reference_center_x_m", "reference_center_y_m",
        "reference_center_displacement_m", "left_target_rad_s", "right_target_rad_s",

        // GPS、陀螺仪与左前轮心的四组三维向量。
        "gps_x_m", "gps_y_m", "gps_z_m", "gps_vx_m_s", "gps_vy_m_s", "gps_vz_m_s", "gyro_x_rad_s",
        "gyro_y_rad_s", "gyro_z_rad_s", "lf_wheel_x_base_m", "lf_wheel_y_base_m",
        "lf_wheel_z_base_m"};

    for (const auto* leg : LEGS) {
        names.push_back(std::string(leg) + "_angle_rad");
        names.push_back(std::string(leg) + "_speed_rad_s");
        for (const auto* joint : LEG_JOINTS) {
            const std::string joint_name = std::string(leg) + "_" + joint;
            for (const auto* suffix : {"_target_rad", "_angle_rad", "_error_rad", "_speed_rad_s"}) {
                names.push_back(joint_name + suffix);
            }
        }
    }
    // 新列全部追加，已有反馈列的位置保持不变。
    names.push_back("wheel_control_mode");
    for (const auto* leg : LEGS) {
        names.push_back(std::string(leg) + "_wheel_hold_target_rad");
    }
    return names;
}

ExperimentRecorder::ExperimentRecorder(const std::filesystem::path& folder,
                                       const std::string& metadata)
    : last_print_time_(-std::numeric_limits<double>::infinity()) {
    std::filesystem::create_directories(folder);
    path_ = folder / ("feedback_diag_" + timestampForFilename() + ".csv");
    metadata_path_ = path_;
    metadata_path_.replace_extension(".meta.json");

    std::ofstream metadata_file;
    metadata_file.exceptions(std::ios::failbit | std::ios::badbit);
    metadata_file.open(metadata_path_, std::ios::binary);
    metadata_file << metadata << '\n';
    metadata_file.close();

    file_.exceptions(std::ios::failbit | std::ios::badbit);
    file_.open(path_, std::ios::binary);
    file_ << "\xEF\xBB\xBF";  // 与 Python 的 utf-8-sig CSV 一致。
    writeCsvRow(file_, fields());
    file_.flush();
}

// ~ 表示析构函数：对象离开作用域时自动释放文件资源，这是 RAII 的用法。
ExperimentRecorder::~ExperimentRecorder() {
    try {
        close();
    } catch (...) {
        // 析构函数不能再次抛异常。正常结束时主循环已显式调用 close()。
    }
}

void ExperimentRecorder::close() {
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

void ExperimentRecorder::record(const SensorSample& feedback, WheelCommand previous_command,
                                const std::optional<WheelValues>& previous_hold_angles) {
    // 每个有效采样都写 CSV，控制台摘要按周期打印；打印周期不等于采样周期。
    writeCsvRow(file_, makeDataRow(feedback, previous_command, previous_hold_angles));

    const bool print_due = feedback.time_s - last_print_time_ >= PRINT_PERIOD_SECONDS;
    if (print_due) {
        printFeedbackSummary(feedback, previous_command, previous_hold_angles);
        file_.flush();
        last_print_time_ = feedback.time_s;
    }
}

}  // namespace wl
