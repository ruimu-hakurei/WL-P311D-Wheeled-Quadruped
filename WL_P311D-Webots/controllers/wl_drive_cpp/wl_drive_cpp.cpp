// 【入口 / 调度层】按仿真节拍组织读取、计算、记录和执行。
// 阅读路线：main → runController → 当前模式的控制器 → RobotIO。
// 主循环负责切换模式和选择执行目标。各模式的运动算法分别放在各自的模块。
#include "config.hpp"
#include "kinematics.hpp"
#include "recorder.hpp"
#include "robot_io.hpp"
#include "rolling.hpp"
#include "run_options.hpp"
#include "body_control.hpp"
#include "estimation.hpp"
#include "crawl_controller.hpp"
#include "trot_controller.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>

// 匿名命名空间让这些辅助函数仅供本源文件使用，避免和其他 .cpp 的名字冲突。
namespace {

constexpr double TROT_FORWARD_M_S = 0.090;  // 换算成步幅的前进输入，单位 m/s。
constexpr double TROT_ARC_STEP_RAD = 0.12;  // 边走边转的单次换步规划转角，单位 rad。
constexpr double TROT_PIVOT_STEP_RAD = 0.18;

// 轮式和爬行共用 W/S/A/D 的解码结果，小跑在此转成自己使用的物理量。
struct TrotCommand {
    double forward_m_s = -1.0;  // 负值请求停步，0 允许原地转。
    double yaw_step_rad = 0.0;  // 单次换步的转角输入，正值左转。
};

TrotCommand makeTrotCommand(const wl::MotionCommand& command) {
    const bool forward = command.forward > 0.0;
    if (!forward && command.turn == 0.0) {
        return {};  // 按结构体默认值构造停步指令。
    }
    const double turn_step_rad = forward ? TROT_ARC_STEP_RAD : TROT_PIVOT_STEP_RAD;
    // 除以轮式转弯档位得到方向系数，再乘小跑的单步转角。
    return {forward ? TROT_FORWARD_M_S : 0.0,
            turn_step_rad * command.turn / wl::TURN_SPEED};
}

bool hasDirectionKey(const std::set<int>& keys) {
    // 检查原始按键，空格和相反方向同时按下时，解码后的运动量可能为零。
    return keys.count('W') || keys.count('S') || keys.count('A') || keys.count('D')
           || keys.count(webots::Keyboard::UP) || keys.count(webots::Keyboard::DOWN)
           || keys.count(webots::Keyboard::LEFT) || keys.count(webots::Keyboard::RIGHT);
}

webots::Keyboard& initializeKeyboard(webots::Robot& robot, int time_step_ms) {
    // 解码函数可以独立于 Webots 计算；这里确认 SDK 的方向键编号未变化。
    static_assert(webots::Keyboard::LEFT == 314 && webots::Keyboard::UP == 315
                      && webots::Keyboard::RIGHT == 316 && webots::Keyboard::DOWN == 317,
                  "Update the keyboard decoder for this SDK");

    // auto* 是推断出的设备指针；返回引用前先排除 nullptr，所有权仍属于 Webots。
    auto* keyboard = robot.getKeyboard();
    if (keyboard == nullptr) {
        throw std::runtime_error("Keyboard unavailable");
    }
    keyboard->enable(time_step_ms);
    return *keyboard;
}

std::set<int> readPressedKeys(webots::Keyboard& keyboard) {
    std::set<int> pressed_keys;
    int key = keyboard.getKey();
    // 读完本周期按键队列，set 自动去重；KEY 掩码提取键码，去掉修饰键标志。
    while (key != -1) {
        pressed_keys.insert(key & webots::Keyboard::KEY);
        key = keyboard.getKey();
    }
    return pressed_keys;
}

// 根据同一帧反馈，为左前腿准备抬起计算支撑平移目标。
// 只计算、打印，不发送电机命令。
void printSupportShiftPlan(const wl::FourLegKinematics& kinematics,
                           const wl::SensorSample& feedback) {
    // const& 表示只读引用：
    // 使用已有对象，不复制整份数据，也不修改调用者的数据。
    try {
        // 1. 用当前实测角度冻结本步的四个轮心参考。
        // 必须与下面规划使用的 feedback 来自同一帧。
        const wl::CrawlStepPlan step =
            wl::makeCrawlStepPlan(kinematics, feedback.joint_angles, wl::LEFT_FRONT_LEG_INDEX,
                                  wl::CRAWL_STEP_LENGTH_M, wl::CRAWL_LIFT_HEIGHT_M);

        // 2. 计算机身应相对本步起始参考系平移多少。
        // 返回的是三个位置分量，单位 m，不是三个关节角。
        const wl::Vec3 shift =
            wl::planSupportShift(kinematics, step, feedback, wl::LEG_JOINT_LIMITS);

        // 3. 打印规划结果。
        // << 把后面的内容依次写入输出流；
        // std::endl 换行并刷新，让结果及时显示。
        std::cout << "Support plan LF: body_shift_m=(" << shift[0] << ", " << shift[1] << ", "
                  << shift[2] << ')' << std::endl;
    } catch (const std::exception& error) {
        // 逆解不可达、三角形退化或迭代未收敛时，
        // 下层抛出的异常会来到这里。
        // 这次诊断失败只报告原因，不终止原有站姿控制。
        std::cerr << "Support plan failed: " << error.what() << std::endl;
    }
}

// 支撑裕量使用当前实测姿态和关节角，诊断失败时只输出原因。
void printSupportMargin(const char* phase,
                        const wl::FourLegKinematics& kinematics,
                        const wl::SensorSample& feedback,
                        std::size_t swing_leg) {
    try {
        const double margin_m = wl::estimateSupportMargin(kinematics, feedback, swing_leg);
        std::cout << "Support margin " << phase << ' ' << wl::LEGS[swing_leg]
                  << ": " << margin_m * 1000.0 << " mm" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "Support margin " << phase << " unavailable: "
                  << error.what() << std::endl;
    }
}

// 输出模式事件；调用位置在目标下发之后，Enabled 对应本拍已发送的保持目标。
void printWheelHoldEvent(const wl::WheelHoldGoal& goal) {
    switch (goal.event) {
        case wl::WheelHoldEvent::None:
            return;
        case wl::WheelHoldEvent::Enabled:
            std::cout << "Wheel hold ENABLED: targets_rad=(";
            for (std::size_t leg_index = 0; leg_index < wl::LEG_COUNT; ++leg_index) {
                if (leg_index != 0) {
                    std::cout << ',';
                }
                std::cout << wl::LEGS[leg_index] << ':' << goal.target_angles.value()[leg_index];
            }
            std::cout << "). WASD ignored; R releases." << std::endl;
            return;
        case wl::WheelHoldEvent::Released:
            std::cout << "Wheel hold RELEASED: velocity mode, zero speed this frame." << std::endl;
            return;
        case wl::WheelHoldEvent::FeedbackLost:
            std::cout
                << "Wheel hold EXITED: feedback invalid; zero wheel speed. Press H again after recovery."
                << std::endl;
            return;
        case wl::WheelHoldEvent::RejectedStand:
            std::cout << "Wheel hold REJECTED: wait for Stand=HOLD, then press H again."
                      << std::endl;
            return;
        case wl::WheelHoldEvent::RejectedMotion:
            std::cout
                << "Wheel hold REJECTED: release drive keys and wait for valid, stopped wheel speeds; press H again."
                << std::endl;
            return;
        case wl::WheelHoldEvent::RejectedFeedback:
            std::cout << "Wheel hold REJECTED: valid wheel feedback required; press H again."
                      << std::endl;
            return;
    }
}

void printStandGoal(const wl::StandGoal& goal) {
    const char* phase_name = "UNKNOWN";

    switch (goal.phase) {
        case wl::StandPhase::Prepare:
            phase_name = "PREPARE";
            break;
        case wl::StandPhase::WaitReached:
            phase_name = "WAIT_REACHED";
            break;
        case wl::StandPhase::Hold:
            phase_name = "HOLD";
            break;
    }

    std::cout << "Stand=" << phase_name << " reached=" << goal.reached;

    // 打印四腿目标，便于检查关节顺序和角度范围。
    for (std::size_t leg_index = 0; leg_index < wl::LEG_COUNT; ++leg_index) {
        const auto& angles = goal.target_angles[leg_index];

        std::cout << ' ' << wl::LEGS[leg_index] << "=(" << angles[0] << ',' << angles[1] << ','
                  << angles[2] << ')';
    }

    std::cout << std::endl;
}

// 正常调用链：step → read → 日志/输入 → stand.update → 轮速分配 → applyTargets。
int runController(const wl::RunOptions& options) {
    // 爬行从 Supervisor 读取接地点；隔离校验可在结束时退出仿真器。
    webots::Supervisor robot;
    const int time_step_ms = static_cast<int>(robot.getBasicTimeStep());
    if (options.quit_webots && !robot.getSupervisor()) {
        throw std::runtime_error("--quit-webots requires a supervisor validation world");
    }

    // 持久对象在 while 外创建：设备句柄、站姿过渡、几何参数和航向参考跨帧保留。
    wl::RobotIO robot_io(robot, time_step_ms);
    // StandController 记住首次实测起点和开始时刻；每拍重新构造会不断重置轨迹。
    wl::StandController stand(wl::STAND_TARGET_ANGLES, wl::STAND_DURATION_S,
                              wl::STAND_TOLERANCE_RAD);
    // 单腿对象用于左前轮心 FK 诊断，四腿对象用于整机模型质心计算。
    // const 表示这些计算对象在这里按只读方式使用；它们不保存 Motor 句柄。
    const wl::LegKinematics kinematics(wl::LF_GEOMETRY);
    const wl::FourLegKinematics four_legs_kinematics({wl::LEG_GEOMETRIES});
    wl::HeadingController heading(1.0, 0.5);
    // H/R 只安排轮子保持模式；不会生成机身平移或摆腿目标。
    wl::WheelHoldController wheel_hold(wl::WHEEL_HOLD_ENTRY_SPEED_TOLERANCE_RAD_S);
    auto& keyboard = initializeKeyboard(robot, time_step_ms);

    // auto 由右侧推断类型；这里是 filesystem::path。路径的 / 运算符表示拼接目录。
    const auto project_folder = std::filesystem::u8path(robot.getProjectPath());
    const auto controller_folder = project_folder / "controllers/wl_drive_cpp";
    const auto world_path = std::filesystem::u8path(robot.getWorldPath());
    const std::string metadata = wl::runMetadata(robot_io, time_step_ms, controller_folder,
                                                 world_path,
                                                 options.scripted_input || options.scripted_trot);
    wl::ExperimentRecorder recorder(controller_folder / "logs", metadata);

    const double start_time_s = robot.getTime();
    const double ready_time_s = start_time_s + wl::SETTLE_SECONDS;
    double next_stand_print_s = ready_time_s;
    // 放在 while 外，跨帧记住是否已经尝试打印。
    // 如果放在 while 内，每拍都会重置成 false。
    bool support_plan_printed = false;
    std::size_t valid_frames = 0;
    std::size_t invalid_frames = 0;
    bool finished_by_timer = false;
    wl::TrotValidation validation;

    std::cout
        << "CSV: " << recorder.path().u8string() << '\n'
        << "Metadata: " << recorder.metadataPath().u8string() << '\n'
        << "W/S: drive, A/D: turn, space: stop. H: hold stopped wheels after Stand=HOLD;"
        << " B: try LF support shift up to 3 cm during hold; R: release."
        << "\nG: enter crawl mode during H hold. Tap or hold WASD. Space: stop. R: exit."
        << "\nT: enter diagonal trot during H hold. Hold W/A/D. Space: stop; R: recenter and exit."
        << std::endl;

    // 保存 B 键按下那一拍的实测起点，供整个平移过程使用。
    struct BodyShiftStart {
        wl::CrawlStepPlan step;
        wl::SensorSample feedback;
        bool end_margin_printed = false;
    };
    std::optional<BodyShiftStart> body_shift_start;
    bool body_shift_key_was_down = false;
    bool crawl_key_was_down = false;
    bool release_key_was_down = false;
    bool crawl_mode = false;
    bool crawl_has_targets = false;
    bool crawl_motion_was_down = false;
    bool crawl_motion_blocked = false;
    std::optional<wl::CrawlController> crawl;
    // optional 的有值状态就是小跑模式。emplace 原地构造，reset 销毁并清空。
    std::optional<wl::TrotController> trot;
    bool trot_key_was_down = false;
    bool trot_exit_requested = false;
    bool trot_motion_blocked = false;

    // 一次循环对应一个仿真采样周期；step 返回 -1 表示连接结束。
    // ms 用于 SDK 采样周期，后续算法时间 now_s 则使用秒，不能混用。
    while (robot.step(time_step_ms) != -1) {
        const double now_s = robot.getTime();

        // 1. 读取本拍反馈。optional 没有值时，本拍反馈不能用于控制。
        // optional 类似“可能为空的数据盒”：先 has_value()，再用 -> 访问成员。
        // feedback 是本拍的值快照：后续给不同算法传同一份反馈，可避免输入时刻不一致。
        // SensorSample::joint_angles 是实测；控制器输出的 target_angles 是目标，不能互换。
        auto feedback = robot_io.read(now_s);
        if (feedback.has_value()) {
            ++valid_frames;
            feedback->lf_wheel_center_base_m =
                kinematics.forward(feedback->joint_angles[wl::LEFT_FRONT_LEG_INDEX]);

            // 2. 反馈来自上一执行区间，先记录上一拍实际下发的轮速。
            // 两份目标记录来自上一执行区间；此处尚未发送本拍的新目标。
            recorder.record(feedback.value(), robot_io.appliedCommand(),
                            robot_io.appliedWheelHoldAngles());
        } else {
            ++invalid_frames;
            if (trot) {
                // 留住最后一帧轮角保持与腿角。反馈恢复后用 R 请求回中。
                trot->onFeedbackLost();
                if (options.scripted_trot) validation.first_fault = trot->firstFault();
                if (wl::shouldFinishValidation(options, now_s - start_time_s)) {
                    finished_by_timer = true;
                    break;
                }
                continue;
            }
        }

        // 3. 正常运行读取键盘；显式验证模式使用同一解码器处理脚本按键。
        const auto pressed_keys = options.scripted_trot
                                      ? wl::scriptedKeysForTrotValidation(now_s - start_time_s)
                                  : options.scripted_input
                                      ? wl::scriptedKeysForValidation(now_s - start_time_s)
                                      : readPressedKeys(keyboard);
        auto command = wl::decodeKeyboardCommand(pressed_keys);

        // 4. 反馈有效且启动等待结束后，更新四腿姿态目标。
        // 这是每拍新建的可选输出；无反馈/未就绪时保持为空，不沿用上一拍的局部目标。
        std::optional<wl::StandGoal> stand_goal;

        if (feedback.has_value() && now_s >= ready_time_s) {
            // stand 是循环外的持久控制器；stand_goal 是本拍重新生成的结果。
            // feedback-> 访问 optional 内部的样本；当前 if 已保证它确实有值。
            stand_goal = stand.update(now_s, feedback->joint_angles);
            // 第一次实际到达站姿时，计算并打印一次支撑准备目标。
            // && 左侧为 false 时，不再检查右侧条件。
            if (!support_plan_printed && stand_goal->phase == wl::StandPhase::Hold) {
                // feedback.value() 取得 optional 内的样本。
                // 外层 if 已经保证 feedback 有值。
                printSupportShiftPlan(four_legs_kinematics, feedback.value());

                // 无论规划成功还是打印了失败原因，本次运行都只尝试一次。
                support_plan_printed = true;
            }

            if (now_s >= next_stand_print_s) {
                printStandGoal(stand_goal.value());

                // 模型质心：实测角度 + URDF 质量参数，结果在 base 中；不是 GPS 世界位置。
                // 这里只按打印周期计算，不作为当前行驶控制的反馈量。
                // 这里传实测角度，打印“当前姿态的模型质心”。
                // planSupportShift 内传候选角度，计算的是“假设平移后的模型质心”，用途不同。
                const wl::Vec3 com_base_m = wl::estimateCenterOfMass(
                    four_legs_kinematics, feedback->joint_angles, feedback->angles);

                std::cout << "Model COM base (m)=(" << com_base_m[0] << ", " << com_base_m[1]
                          << ", " << com_base_m[2] << ')' << std::endl;

                next_stand_print_s = now_s + wl::PRINT_PERIOD_SECONDS;
            }
        }

        // 5. 四腿实际到位后，才允许使用键盘行驶指令。
        // Hold 每拍重新判断，受扰动超出站姿容差时会回到 WaitReached，行驶随之暂停。
        // “关节到位”并不意味着已经检测到接地；这里还是原有站姿行驶门控。
        // && 从左向右短路求值：没有 stand_goal 时，不会继续通过 -> 读取它的 phase。
        // 是否允许行驶仍按站姿判断；没有把支撑规划是否收敛混进当前控制门控。
        const bool drive_allowed =
            stand_goal.has_value() && stand_goal->phase == wl::StandPhase::Hold;

        // 使用原始键盘运动请求检查是否允许进入；反馈无效时状态机会清空旧保持目标。
        auto wheel_keys = pressed_keys;
        const bool release_key_down = pressed_keys.count('R') != 0;
        const bool release_requested = release_key_down && !release_key_was_down;
        release_key_was_down = release_key_down;
        bool trot_just_exited = false;
        if (trot) {
            // 保留一次 R 的退出请求。回中期间继续保持轮角，确认站稳后才释放。
            trot_exit_requested |= release_requested;
            wheel_keys.erase('R');
            if (trot->readyToExit()) {
                wheel_keys.insert('R');
                if (options.scripted_trot) validation.exited = true;
                trot.reset();
                trot_exit_requested = false;
                trot_just_exited = true;
                std::cout << "Trot mode exited." << std::endl;
            }
        }
        // 爬行中按 R 只请求停车。松开并再按 R 才释放轮角保持。
        if (crawl_mode) {
            if (!release_requested || crawl->running()) {
                wheel_keys.erase('R');
            }
            if (release_requested || pressed_keys.count(' ') != 0) {
                crawl->requestStop();
            }
        }
        const wl::WheelHoldGoal wheel_hold_goal =
            wheel_hold.update(wheel_keys, feedback, drive_allowed, command);

        const bool crawl_key_down = pressed_keys.count('G') != 0;
        const bool crawl_requested = crawl_key_down && !crawl_key_was_down;
        crawl_key_was_down = crawl_key_down;
        const bool trot_key_down = pressed_keys.count('T') != 0;
        const bool trot_requested = trot_key_down && !trot_key_was_down;
        trot_key_was_down = trot_key_down;

        if (!wheel_hold_goal.target_angles) {
            if (crawl_mode) {
                if (crawl->running()) {
                    crawl->hold("feedback unavailable");
                }
                crawl->clearPausedStep();
                std::cout << "Crawl mode exited." << std::endl;
            }
            crawl_mode = false;
            crawl_has_targets = false;
        }
        if (crawl_requested && !crawl_mode && !trot) {
            if (!drive_allowed || !wheel_hold_goal.target_angles || body_shift_start) {
                std::cout << "Crawl requires Stand=HOLD and H before G." << std::endl;
            } else if (!robot.getSupervisor()) {
                std::cout << "Crawl requires Supervisor contact feedback." << std::endl;
            } else {
                try {
                    if (!crawl) {
                        crawl.emplace(robot);
                    }
                    crawl_mode = true;
                    std::cout << "Crawl mode ready. Tap or hold W, S, A or D." << std::endl;
                } catch (const std::exception& error) {
                    std::cerr << "Crawl start rejected: " << error.what() << std::endl;
                }
            }
        }

        if (trot_requested && !trot) {
            if (!drive_allowed || !wheel_hold_goal.target_angles || body_shift_start || crawl_mode) {
                std::cout << "Trot requires Stand=HOLD and H before T." << std::endl;
            } else if (!robot.getSupervisor()) {
                std::cout << "Trot requires Supervisor contact feedback." << std::endl;
            } else {
                try {
                    trot.emplace(robot, four_legs_kinematics, options.scripted_trot);
                    trot_motion_blocked = true;
                    std::cout << "Trot mode ready. Hold W/A/D. Space: stop; R: recenter and exit."
                              << std::endl;
                } catch (const std::exception& error) {
                    std::cerr << "Trot start rejected: " << error.what() << std::endl;
                }
            }
        }
        // 先保存步态指令，后面的轮角保持逻辑会把轮式速度指令清零。
        TrotCommand trot_command;
        if (trot) {
            const bool stop = trot_exit_requested || pressed_keys.count(' ') != 0;
            // 空格停步后保持锁定，直到方向键全部松开。
            if (!hasDirectionKey(pressed_keys)) {
                trot_motion_blocked = false;
            } else if (stop) {
                trot_motion_blocked = true;
            }
            if (!stop && !trot_motion_blocked) {
                trot_command = makeTrotCommand(command);
            }
        }

        if (crawl_mode) {
            const bool stop_key_down = pressed_keys.count(' ') != 0 || release_key_down;
            // 停车后，所有方向键都松开才允许再次起步。
            const bool direction_key_down = hasDirectionKey(pressed_keys);
            const bool motion_down = command.forward != 0.0 || command.turn != 0.0;
            if (!direction_key_down) {
                crawl_motion_blocked = false;
            } else if (stop_key_down || crawl_requested) {
                crawl_motion_blocked = true;
            }

            // 点按从静止开始一轮。长按只在上一轮成功回中后接续。
            const bool new_press = motion_down && !crawl_motion_was_down;
            const bool repeat = motion_down && crawl_has_targets && crawl->cycleCompleted();
            if (!crawl->running() && !stop_key_down && !crawl_motion_blocked
                && (new_press || repeat)) {
                try {
                    crawl->start(four_legs_kinematics, *feedback, command);
                    crawl_has_targets = true;
                } catch (const std::exception& error) {
                    crawl_motion_blocked = true;
                    std::cerr << "Crawl start rejected: " << error.what() << std::endl;
                }
            }
            crawl_motion_was_down = motion_down;
        } else {
            crawl_motion_was_down = false;
            crawl_motion_blocked = false;
        }

        // 只响应 B 从松开到按下的那一拍，长按时起点不会反复重置。
        const bool body_shift_key_down = pressed_keys.count('B') != 0;
        const bool body_shift_requested =
            body_shift_key_down && !body_shift_key_was_down;
        body_shift_key_was_down = body_shift_key_down;

        // R 释放保持或反馈失效后，清掉这次平移的起点。
        if (!wheel_hold_goal.target_angles.has_value()) {
            body_shift_start.reset();
        } else if (body_shift_requested && drive_allowed && !body_shift_start && !crawl_mode && !trot) {
            // 到位条件只在开始时检查，平移途中腿角会离开原站姿。
            const auto& start = feedback.value();
            try {
                auto step = wl::makeCrawlStepPlan(
                    four_legs_kinematics, start.joint_angles, wl::LEFT_FRONT_LEG_INDEX,
                    0.0, 0.0);
                auto shift = wl::planSupportShift(
                    four_legs_kinematics, step, start, wl::LEG_JOINT_LIMITS);

                // 第一次只验证规划方向，按比例缩短位移，保留方向和各分量比例。
                constexpr double MAX_TEST_SHIFT_M = 0.03;
                const double distance = std::hypot(shift[0], shift[1], shift[2]);
                if (distance > MAX_TEST_SHIFT_M) {
                    for (double& component : shift) {
                        component *= MAX_TEST_SHIFT_M / distance;
                    }
                }
                step.body_shift_m = shift;
                body_shift_start = BodyShiftStart{step, start};
                printSupportMargin("before", four_legs_kinematics, start, step.active_leg);
                std::cout << "Body shift started: LF target_m=(" << shift[0] << ", "
                          << shift[1] << ", " << shift[2] << ')' << std::endl;
            } catch (const std::exception& error) {
                std::cerr << "Body shift rejected: " << error.what() << std::endl;
            }
        }

        if (!drive_allowed || wheel_hold_goal.target_angles.has_value()
            || pressed_keys.count('R') != 0 || trot_just_exited) {
            // 保持期间屏蔽 WASD，避免给执行层同时传入保持角度和非零轮速。
            // 按 R 的这一拍先发送零轮速，下一拍才恢复正常按键行驶。
            // 准备、等待到位或反馈无效时，轮速目标也为零。
            command = {};
            heading.reset(std::nullopt);
        } else if (command.turn != 0.0 || command.forward == 0.0) {
            // 手动转弯或停车时，更新航向参考。
            heading.reset(feedback->yaw_rad);
        } else {
            // 直行时，根据航向误差修正转向量。
            command.turn = heading.update(feedback->yaw_rad);
        }

        // 6. 将腿部目标和轮速打包，由设备层统一检查、发送。
        if (stand_goal.has_value()) {
            // command 是前进量、转向量，先转换成左右轮速。
            const wl::WheelCommand wheel_speeds = wl::differentialWheelCommand(
                command.forward, command.turn, robot_io.maxWheelSpeed());

            // 把本拍四腿站姿角度与左右轮速放进同一份值对象。
            // 这一行只构造目标，下面的 applyTargets 才负责校验和实际下发。
            wl::ActuatorTargets targets(stand_goal->target_angles, wheel_speeds);
            // 普通 H 保持使用进入时冻结的轮角。
            targets.wheel_hold_angles = wheel_hold_goal.target_angles;

            // 平移期间由同一个采样结果给出腿角与补偿后的轮角。
            if (trot) {
                // *feedback 解引用本帧快照。状态机产出腿角和轮角，最后统一交给设备层。
                targets = trot->update(*feedback, trot_command.forward_m_s, trot_command.yaw_step_rad,
                                       trot_exit_requested);
                if (options.scripted_trot) {
                    validation.landed_steps = trot->landedSteps();
                    validation.first_fault = trot->firstFault();
                }
            } else if (crawl_mode && crawl_has_targets) {
                targets = crawl->update(four_legs_kinematics, *feedback);
            } else if (body_shift_start.has_value()) {
                auto& start = *body_shift_start;
                const double elapsed_s = now_s - start.feedback.time_s;
                try {
                    targets = wl::sampleBodyShiftTargets(
                        four_legs_kinematics, start.step, start.feedback,
                        wl::LEG_JOINT_LIMITS, elapsed_s);

                    // 轨迹走完且实测腿角接近终点时，只打印一次裕量。
                    // 沿用站姿的关节到位容差，反馈来自本拍而非起始快照。
                    if (!start.end_margin_printed && elapsed_s >= wl::BODY_SHIFT_DURATION_S
                        && wl::legPosesReached(feedback->joint_angles, targets.leg_angles,
                                               wl::STAND_TOLERANCE_RAD)) {
                        start.end_margin_printed = true;
                        printSupportMargin("after", four_legs_kinematics, *feedback,
                                           start.step.active_leg);
                    }
                } catch (const std::exception& error) {
                    std::cerr << "Body shift canceled: " << error.what() << std::endl;
                    body_shift_start.reset();
                }
            }

            // 算法层只组装数据；Motor::setPosition / setVelocity 集中在设备层执行。
            robot_io.applyTargets(targets);
        } else {
            // 尚无有效姿态目标时，保持轮速为零，不生成新的腿部命令。
            robot_io.drive(0.0, 0.0);
        }

        if (options.scripted_trot && feedback) {
            validation.observe(*feedback, pressed_keys, trot.has_value(),
                               !robot_io.appliedWheelHoldAngles(), now_s - start_time_s);
        }
        printWheelHoldEvent(wheel_hold_goal);

        if (wl::shouldFinishValidation(options, now_s - start_time_s)) {
            finished_by_timer = true;
            break;
        }
    }

    // step() 返回 -1 后不再访问设备；定时验证结束时还保持连接，可以发零轮速。
    if (finished_by_timer) {
        if (!trot) {
            robot_io.drive(0.0, 0.0);
        }
    }
    recorder.close();

    if (finished_by_timer) {
        wl::writeValidationCompletion(controller_folder, valid_frames, invalid_frames,
                                      robot.getTime(), options.scripted_trot ? &validation : nullptr);
    }
    std::cout << "Controller finished: valid_frames=" << valid_frames
              << " invalid_frames=" << invalid_frames << std::endl;

    const int status = options.scripted_trot && (!finished_by_timer || !validation.passed(invalid_frames)) ? 1 : 0;
    if (finished_by_timer && options.quit_webots) {
        robot.simulationQuit(status);
    }
    return status;
}

}  // namespace

// 程序入口：argc 是参数数量，argv 是参数字符串数组，argv[0] 通常是程序名。
// const T& 接收只读引用；catch 捕获下层异常，统一报告后以非零状态结束。
int main(int argc, char** argv) {
    try {
        const wl::RunOptions options = wl::parseRunOptions(argc, argv);
        return runController(options);
    } catch (const std::exception& error) {
        std::cerr << "wl_drive_cpp: " << error.what() << std::endl;
        return 1;
    }
}
