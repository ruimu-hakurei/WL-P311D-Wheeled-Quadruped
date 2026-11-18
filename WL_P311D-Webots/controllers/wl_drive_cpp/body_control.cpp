// 【机身协调实现】站姿：按关节空间平滑过渡；迈步：轮心轨迹 + 四腿逆解。
// 支撑三角形中心是几何参考，不等于已经确认机器人支撑稳定。
#include "body_control.hpp"
#include "config.hpp"
#include "estimation.hpp"
#include "motion.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace wl {

bool legPosesReached(const LegPoses& measured, const LegPoses& target, double tolerance_rad) {
    // 逐条腿检查，每条腿包含 HAA、HFE、KFE 三个关节。
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        const bool reached =
            jointPoseReached(measured[leg_index], target[leg_index], tolerance_rad);

        // 任意一条腿没有到位，整机姿态就还没有到位。
        if (!reached) {
            return false;
        }
    }

    // 四条腿全部通过检查。
    return true;
}

StandTransition::StandTransition(const LegPoses& measured, const LegPoses& target,
                                 double start_time_s, double duration_s)
    : transitions_{JointTransition(measured[0], target[0], start_time_s, duration_s),
                   JointTransition(measured[1], target[1], start_time_s, duration_s),
                   JointTransition(measured[2], target[2], start_time_s, duration_s),
                   JointTransition(measured[3], target[3], start_time_s, duration_s)} {
    // JointTransition 构造时必须取得起终点，因此按四腿顺序显式创建四个对象。
}

LegPoses StandTransition::sample(double now_s) const {
    LegPoses target_angles{};
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        target_angles[leg_index] = transitions_[leg_index].sample(now_s);
    }
    return target_angles;
}

StandController::StandController(const LegPoses& target_angles, double duration_s,
                                 double tolerance_rad)
    : target_angles_(target_angles), duration_s_(duration_s), tolerance_rad_(tolerance_rad) {
    // 时间和误差容限都必须是有限的正数。
    if (!std::isfinite(duration_s_) || duration_s_ <= 0.0 || !std::isfinite(tolerance_rad_)
        || tolerance_rad_ <= 0.0) {
        throw std::invalid_argument("Stand duration and tolerance must be finite and positive");
    }

    // 最终目标的全部关节角都必须有效。
    // 关节上下限仍由 RobotIO 在发送前检查。
    for (const auto& leg_angles : target_angles_) {
        for (double angle : leg_angles) {
            if (!std::isfinite(angle)) {
                throw std::invalid_argument("Stand targets must be finite");
            }
        }
    }
}

StandGoal StandController::update(double now_s, const LegPoses& measured) {
    // 时间必须有效。
    if (!std::isfinite(now_s)) {
        throw std::invalid_argument("Stand update time must be finite");
    }

    // 第一次更新：用实测角度作为起点，创建四腿过渡轨迹。
    if (!transition_.has_value()) {
        // emplace 在 optional 内直接构造对象；之后通过 -> 调 sample，不需要裸 new/delete。
        transition_.emplace(measured, target_angles_, now_s, duration_s_);
        start_time_s_ = now_s;
    }

    const double elapsed_s = now_s - start_time_s_;

    // 判断实际关节是否已经到达最终目标。
    const bool reached = legPosesReached(measured, target_angles_, tolerance_rad_);

    // 轨迹还没结束：继续输出平滑过渡中的目标角度。
    if (elapsed_s < duration_s_) {
        return StandGoal{StandPhase::Prepare, transition_->sample(now_s), reached};
    }

    // 轨迹结束：实际到位则保持，否则继续等待到位。
    // ?: 是条件表达式；Hold 不会永久锁存，下一拍实测误差超限会重新等待。
    const StandPhase phase = reached ? StandPhase::Hold : StandPhase::WaitReached;

    return StandGoal{phase, target_angles_, reached};
}

ActuatorTargets buildCrawlTargets(const GaitGoal& goal, const CrawlStepPlan& step,
                                  const FourLegKinematics& kinematics, const LegJointLimits& limits,
                                  const LegPoses& reference_angles) {
    if (goal.active_leg >= LEG_COUNT) {
        throw std::out_of_range("Invalid active leg");
    }
    if (goal.active_leg != step.active_leg) {
        throw std::invalid_argument("Gait goal and step plan refer to different legs");
    }

    for (double component : step.body_shift_m) {
        if (!std::isfinite(component)) {
            throw std::invalid_argument("Body shift must be finite");
        }
    }

    if (!std::isfinite(step.lift_height_m) || step.lift_height_m < 0.0) {
        throw std::invalid_argument("Lift height must be finite and nonnegative");
    }

    // 1. 默认保持本步开始时的四个轮心位置。
    // 复制冻结的轮心参考，再修改当前目标；不能改写 step，否则会污染本步起点。
    WheelCenters targets = step.reference_centers_m;

    // 2. 摆动期间，只更新当前摆动腿的轮心目标。
    if (goal.phase == GaitPhase::Swing) {
        targets[goal.active_leg] = swingPosition(step.swing_start_m, step.swing_end_m,
                                                 goal.swing_progress, step.lift_height_m);
    } else if (goal.phase == GaitPhase::WaitContact) {
        // 等待落地时，保持轨迹终点。
        targets[goal.active_leg] = step.swing_end_m;
    }

    // 本版只处理平移、坐标轴保持平行：机身向 +X 移动，固定支撑点在机身中就向 -X 移动。
    // 这里没有加入机身旋转；改变姿态时需要完整坐标变换，不能继续只减位移。
    // 3. 将参考系中的轮心目标，转换到目标机身坐标系。
    for (auto& position : targets) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            position[axis] -= step.body_shift_m[axis];
        }
    }

    // 4. 求四条腿的关节目标。
    const LegPoses angles = kinematics.inverse(targets, limits, reference_angles);

    // 5. 打包目标，左右轮速默认均为零。
    // 默认轮速为零仅是速度目标；实际轮子位置保持还需要后续独立实现。
    return ActuatorTargets(angles);
}

CrawlStepPlan makeCrawlStepPlan(const FourLegKinematics& kinematics,
                                const LegPoses& measured_angles, std::size_t active_leg,
                                double step_length_m, double lift_height_m) {
    if (active_leg >= LEG_COUNT) {
        throw std::out_of_range("Invalid active leg");
    }

    // 步长允许为负，表示向机身后方迈步。
    if (!std::isfinite(step_length_m) || !std::isfinite(lift_height_m) || lift_height_m < 0.0) {
        throw std::invalid_argument("Invalid step length or lift height");
    }

    CrawlStepPlan step;
    step.active_leg = active_leg;

    // 根据本步开始时的实测角度，保存四个轮心位置。
    step.reference_centers_m = kinematics.forward(measured_angles);

    // 当前腿从自己的轮心位置开始摆动。
    step.swing_start_m = step.reference_centers_m[active_leg];

    // 第一版沿本步参考系的 X 轴确定落点，落点高度保持不变。
    step.swing_end_m = step.swing_start_m;
    step.swing_end_m[0] += step_length_m;

    step.lift_height_m = lift_height_m;

    // body_shift_m 默认是零，后续由支撑准备过程更新。
    return step;
}

Vec3 supportTriangleCenter(const std::array<Vec3, LEG_COUNT>& contact_points_m,
                           std::size_t swing_leg) {
    if (swing_leg >= LEG_COUNT) {
        throw std::out_of_range("Invalid swing leg");
    }

    // 1. 排除即将抬起的腿，保存其余三个接地点。
    std::array<Vec3, 3> support_points{};
    std::size_t count = 0;

    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        if (leg_index == swing_leg) {
            continue;
        }

        const Vec3& point = contact_points_m[leg_index];

        for (double component : point) {
            if (!std::isfinite(component)) {
                throw std::invalid_argument("Contact points must be finite");
            }
        }

        support_points[count++] = point;
    }

    // 2. 计算中心，以及从第一个点出发的两条边。
    Vec3 center{};
    Vec3 edge1{};
    Vec3 edge2{};

    for (std::size_t axis = 0; axis < 3; ++axis) {
        center[axis] =
            (support_points[0][axis] + support_points[1][axis] + support_points[2][axis]) / 3.0;

        edge1[axis] = support_points[1][axis] - support_points[0][axis];
        edge2[axis] = support_points[2][axis] - support_points[0][axis];
    }

    // 3. 叉积长度等于三角形面积的两倍。
    const Vec3 normal{edge1[1] * edge2[2] - edge1[2] * edge2[1],
                      edge1[2] * edge2[0] - edge1[0] * edge2[2],
                      edge1[0] * edge2[1] - edge1[1] * edge2[0]};

    const double double_area = std::hypot(normal[0], normal[1], normal[2]);

    // 数值退化检查：重合或近乎共线的点不能形成有效三角形。
    constexpr double AREA_EPSILON_M2 = 1e-12;
    if (double_area <= AREA_EPSILON_M2) {
        throw std::domain_error("Degenerate support triangle");
    }

    // 返回的是三个点的几何重心：不含整机质量、CoM 投影或稳定裕量判断。
    // 上面的面积阈值只排除数值退化，不能当作允许抬腿的稳定性条件。
    return center;
}

double signedSupportMarginXY(const std::array<Vec3, 3>& triangle,
                             const Vec3& com_projection) {
    // 计算点 p 在有向边 a → b 的哪一侧。
    const auto crossXY = [](const Vec3& a, const Vec3& b, const Vec3& p) {
        return (b[0] - a[0]) * (p[1] - a[1])
             - (b[1] - a[1]) * (p[0] - a[0]);
    };

    const double area_twice = crossXY(triangle[0], triangle[1], triangle[2]);
    if (std::abs(area_twice) <= 1e-12) {
        throw std::domain_error("Degenerate support triangle");
    }

    // 统一顺时针和逆时针的符号，让三角形内侧总为正。
    const double orientation = area_twice > 0.0 ? 1.0 : -1.0;
    std::array<double, 3> distances_m{};

    for (std::size_t i = 0; i < triangle.size(); ++i) {
        const Vec3& a = triangle[i];
        const Vec3& b = triangle[(i + 1) % triangle.size()];
        const double edge_length_m = std::hypot(b[0] - a[0], b[1] - a[1]);

        distances_m[i] =
            orientation * crossXY(a, b, com_projection) / edge_length_m;
    }

    return *std::min_element(distances_m.begin(), distances_m.end());
}

namespace {

Vec3 projectToHorizontal(const Vec3& point_base,
                         double roll_rad, double pitch_rad) {
    // 先绕 X 轴处理横滚，得到旋转后的 y 和 z。
    const double y = std::cos(roll_rad) * point_base[1]
                   - std::sin(roll_rad) * point_base[2];
    const double z = std::sin(roll_rad) * point_base[1]
                   + std::cos(roll_rad) * point_base[2];

    // 再绕 Y 轴处理俯仰，得到水平坐标系中的 x。
    const double x = std::cos(pitch_rad) * point_base[0]
                   + std::sin(pitch_rad) * z;

    return {x, y, 0.0};
}

}  // namespace

double estimateSupportMargin(const FourLegKinematics& kinematics,
                             const SensorSample& feedback,
                             std::size_t swing_leg) {
    if (swing_leg >= LEG_COUNT) {
        throw std::out_of_range("Invalid swing leg");
    }

    const auto contacts = estimateWheelContactPoints(kinematics, feedback);
    const Vec3 com_base = estimateCenterOfMass(
        kinematics, feedback.joint_angles, feedback.angles);

    std::array<Vec3, 3> triangle{};
    std::size_t support_index = 0;
    for (std::size_t leg_index = 0; leg_index < LEG_COUNT; ++leg_index) {
        if (leg_index == swing_leg) {
            continue;
        }

        triangle[support_index] = projectToHorizontal(
            contacts[leg_index], feedback.roll_rad, feedback.pitch_rad);
        ++support_index;
    }

    const Vec3 com_projection = projectToHorizontal(
        com_base, feedback.roll_rad, feedback.pitch_rad);

    return signedSupportMarginXY(triangle, com_projection);
}

// 返回机身相对本步起始参考系的平移目标，单位 m。
// 假设水平地面、机身姿态不变、四个轮心固定；只做模型规划。
// 阅读本函数时，始终分清两个坐标系：
//   起始参考系：本步开始时固定下来，支撑中心和返回的 shift 都在这里表达。
//   候选机身系：假设机身平移 shift 后的新坐标系；逆解后的模型质心先在这里表达。
// 本版两者坐标轴平行，只有原点平移，所以可以直接用“候选坐标 + shift”转换。
// 若以后同时改变机身姿态，这里必须加入旋转变换，不能沿用仅相加/相减的写法。
// 调度层可在支撑平移开始时调用；此处只计算计划。
Vec3 planSupportShift(const FourLegKinematics& kinematics, const CrawlStepPlan& step,
                      const SensorSample& start_feedback, const LegJointLimits& limits) {
    // constexpr 是编译期常量，放在函数内表示这些设置仅属于这个求解过程。
    // 40 是一次函数调用内部的最多尝试次数，不是执行 40 拍仿真或移动 40 次。
    constexpr int MAX_ITERATIONS = 40;
    // 1 mm 用于候选模型质心的水平投影误差，不是电机跟踪误差或接地检测阈值。
    constexpr double POSITION_TOLERANCE_M = 0.001;
    // 松弛系数决定每轮修正比例；不是 PID 的比例增益，也不是运动速度。
    // 取一半是为了缓和迭代变化，但不保证所有姿态都一定能收敛。
    constexpr double RELAXATION = 0.5;

    // 1. 使用步起始反馈，计算其余三腿的支撑三角形中心。
    // 这些位置都表达在本步开始时的机身参考系中。
    // auto 在这里推断为四个三维接地点的数组。
    // 这些是模型的几何候选点：即使某个轮子悬空，也不能仅凭这个结果判为已接地。
    const auto contacts = estimateWheelContactPoints(kinematics, start_feedback);

    // step.active_leg 是准备抬起的腿编号；中心由另外三个点计算。
    // 当前只是规划“准备抬哪条腿”，并没有命令这条腿离地。
    // 选三角形中心作为简单的内部目标，不是在这里求最优稳定裕量。
    const Vec3 support_center = supportTriangleCenter(contacts, step.active_leg);

    // 世界竖直向上方向，在起始机身参考系中的表达。
    // 机身倾斜时，水平面不等于机身自身的 XY 平面。
    // up 是单位方向向量：机身水平时为 (0, 0, 1)。
    // yaw 围绕世界竖直轴旋转，不改变世界竖直方向，所以这里不需要 yaw。
    // 后续减掉沿 up 的分量，得到的是世界水平面内的误差，仍用起始参考系坐标表示。
    const Vec3 up{-std::sin(start_feedback.pitch_rad),
                  std::cos(start_feedback.pitch_rad) * std::sin(start_feedback.roll_rad),
                  std::cos(start_feedback.pitch_rad) * std::cos(start_feedback.roll_rad)};

    // 2. 从零平移开始尝试。
    // 复制计划，只修改副本，保留调用者保存的本步起点。
    // {} 把三个分量都初始化为零。这里表示尚未尝试机身平移，不是目标世界原点。
    // 从零开始重新求解；并不是把传入 step.body_shift_m 当作本次迭代初值。
    Vec3 shift{};
    // 这是值拷贝：candidate 中的轮心参考保持相同，但它的平移字段可以独立修改。
    // 若改成引用并直接修改原计划，会破坏调用者保存的步起始数据。
    CrawlStepPlan candidate = step;
    // 第一轮靠近实测角度选逆解；后面每轮靠近上一轮候选解选分支。
    // reference_angles 是求解参考，不是已经测量到或已经发送给电机的角度。
    LegPoses reference_angles = start_feedback.joint_angles;

    // 支撑准备期间，四个轮心都保持在本步参考位置。
    // 局部 GaitGoal 只是传给 buildCrawlTargets 的计算条件。
    // 它不会修改 CrawlGait 对象，也不会让整机步态状态机自动切换阶段。
    GaitGoal preparation;
    // enum class 的枚举值写成 GaitPhase::ShiftSupport，明确它属于哪种阶段。
    // 这个阶段保持四个轮心参考；Swing 阶段才会叠加某条腿的摆动轨迹。
    preparation.phase = GaitPhase::ShiftSupport;
    preparation.active_leg = step.active_leg;

    // for 的三个部分：从第 0 轮开始、检查次数上限、每轮结束把计数加一。
    // 所有迭代都使用同一份起始反馈，不是在循环中重新采集不断变化的机器人状态。
    for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
        // 3. 尝试这个机身平移，求对应的四腿关节角。
        // 不可达或没有限位内的解时，下层会抛出异常。
        // 固定点相对移动机身的坐标 = 起始参考位置 - shift。
        // 这段转换由 buildCrawlTargets 完成，这里只传入候选平移。
        candidate.body_shift_m = shift;

        // 返回 ActuatorTargets 仅代表产生了数据；没有调用 RobotIO::applyTargets，电机不会动。
        // 本版遇到一个不可达候选就报告失败，不会自动回退步长或搜索其他移动路径。
        const ActuatorTargets targets =
            buildCrawlTargets(preparation, candidate, kinematics, limits, reference_angles);

        // 4. 腿姿态改变后，重新计算整机质心。
        // com_base 在候选机身坐标系中。
        // 必须使用 targets.leg_angles 重算，而不是一直使用起始实测角度。
        // 轮子转角保持为起始值：这里规划的是轮心固定条件下的机身平移。
        // 重新计算质心是迭代的原因：机身挪动后，各段连杆的姿态与质量分布都会改变。
        const Vec3 com_base =
            estimateCenterOfMass(kinematics, targets.leg_angles, start_feedback.angles);

        // difference 是“支撑中心减去候选质心”，两者必须先统一到起始参考系。
        // 每轮重新清零累加量，避免把上一轮误差错误累积到这一轮。
        Vec3 difference{};
        double vertical_difference = 0.0;

        for (std::size_t axis = 0; axis < 3; ++axis) {
            // 加上 shift，才把质心转换回本步起始参考系。
            difference[axis] = support_center[axis] - (com_base[axis] + shift[axis]);

            // 点积：提取偏差沿世界竖直方向的分量。
            // 三个分量逐项相乘再累加，就是向量点积；+= 表示在当前值上继续累加。
            // 因为 up 已归一化，结果可直接作为竖直投影的有符号长度，单位 m。
            vertical_difference += difference[axis] * up[axis];
        }

        // 5. 去掉竖直分量，只保留水平投影误差。
        // 我们要让质心投影接近支撑中心，不是把质心降到地面。
        // 偏差 = 沿 up 的竖直部分 + 垂直 up 的水平部分。
        // 支撑点接近地面、质心在上方，它们有高度差很正常；这里保留的是水平部分。
        Vec3 error{};

        for (std::size_t axis = 0; axis < 3; ++axis) {
            error[axis] = difference[axis] - vertical_difference * up[axis];
        }

        // hypot 求三维误差的欧氏长度，比较的是距离，不能用三个分量的有符号和代替。
        // error 虽然有三个机身系分量，但整体位于世界水平面内，因此 Z 分量未必是零。
        const double distance = std::hypot(error[0], error[1], error[2]);

        if (distance <= POSITION_TOLERANCE_M) {
            // return 结束整个函数，不仅退出 if 或 for。
            // 返回的是刚才已经通过逆解检查并计算过投影误差的这一组候选平移。
            return shift;
        }

        // 6. 每次修正一半误差，再重新求逆解、重新计算质心。
        // 避免直接按第一次算出的偏差一次移动到位。
        for (std::size_t axis = 0; axis < 3; ++axis) {
            // 正误差让该方向的平移增大，负误差让其减小。
            // 因为腿姿态会随平移变化，这次修正后必须回到循环顶部重新评价。
            shift[axis] += RELAXATION * error[axis];
        }

        // 下一轮选择逆解时，尽量靠近本轮关节姿态。
        // 值赋值保留本轮解供下一轮使用；这里没有改写 start_feedback 的实测角度。
        reference_angles = targets.leg_angles;
    }

    // 达到最大次数仍没有满足误差要求，明确报告规划失败。
    // throw 交给调用层处理：不要把最后一次尚未收敛的 shift 冒充成功结果。
    // 数值上求到终点，也还需要后续检查移动过程、轮子保持与实测支撑条件，才能用于执行。
    throw std::domain_error("Support shift planning did not converge");
}

WheelValues compensateWheelAngles(
    const LegPoses& start_leg_angles,
    const WheelValues& start_wheel_angles,
    const LegPoses& target_leg_angles) {
    WheelValues wheel_targets{};

    for (std::size_t leg_index = 0;
         leg_index < LEG_COUNT;
         ++leg_index) {
        // 1. 检查本腿的起始角度和目标角度。
        for (std::size_t joint_index = 0;
             joint_index < JOINT_COUNT;
             ++joint_index) {
            if (!std::isfinite(
                    start_leg_angles[leg_index][joint_index])
                || !std::isfinite(
                    target_leg_angles[leg_index][joint_index])) {
                throw std::invalid_argument(
                    "Leg angles must be finite");
            }
        }

        if (!std::isfinite(start_wheel_angles[leg_index])) {
            throw std::invalid_argument(
                "Start wheel angles must be finite");
        }

        // 2. 髋俯仰、膝关节和轮子共用随髋侧摆旋转的 Y 轴方向。
        // HAA 绕与轮轴垂直的 X 轴改变轮轴朝向，沿轮轴的自转补偿仍只看 HFE + KFE。
        const double start_pitch =
            start_leg_angles[leg_index][1]
            + start_leg_angles[leg_index][2];

        const double target_pitch =
            target_leg_angles[leg_index][1]
            + target_leg_angles[leg_index][2];

        // 3. 腿部绕轮轴方向多转多少，轮电机相对小腿反向补偿多少。
        // 始终相对固定起点计算，避免逐拍累加产生漂移。
        wheel_targets[leg_index] =
            start_wheel_angles[leg_index]
            + start_pitch
            - target_pitch;

        if (!std::isfinite(wheel_targets[leg_index])) {
            throw std::overflow_error(
                "Compensated wheel angle is not finite");
        }
    }

    return wheel_targets;
}

ActuatorTargets sampleBodyShiftTargets(
    const FourLegKinematics& kinematics,
    const CrawlStepPlan& start_step,
    const SensorSample& start_feedback,
    const LegJointLimits& limits,
    double elapsed_s) {
    // start_step.body_shift_m 保存完整的平移终点，本拍只取轨迹上的中间位置。
    CrawlStepPlan step = start_step;
    step.body_shift_m =
        interpolatePosition(Vec3{}, start_step.body_shift_m, elapsed_s, BODY_SHIFT_DURATION_S);

    // 使用支撑阶段求逆解，四个轮心继续以起始位置为参考。
    GaitGoal goal{};
    goal.phase = GaitPhase::ShiftSupport;
    goal.active_leg = step.active_leg;

    auto targets = buildCrawlTargets(
        goal, step, kinematics, limits, start_feedback.joint_angles);

    // 用起始腿角、起始轮角和本拍的目标腿角计算补偿轮角。
    targets.wheel_hold_angles = compensateWheelAngles(
        start_feedback.joint_angles, start_feedback.angles, targets.leg_angles);

    return targets;
}

}  // namespace wl
