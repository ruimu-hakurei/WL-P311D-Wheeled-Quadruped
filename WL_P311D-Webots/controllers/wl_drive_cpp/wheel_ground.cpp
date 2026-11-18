#include "wheel_ground.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <webots/Field.hpp>

namespace wl {

double flatFloorHeight(webots::Supervisor& robot) {
    auto* floor = robot.getFromDef("WALKING_FLOOR");
    if (!floor) {
        return 0.0;  // 兼容旧版零高度平地世界。
    }
    auto* field = floor->getField("boundingObject");
    auto* collider = field ? field->getSFNode() : nullptr;
    if (!collider || collider->getTypeName() != "Box") {
        throw std::runtime_error("WALKING_FLOOR requires a horizontal Box boundingObject");
    }
    return floor->getPosition()[2] + 0.5 * collider->getField("size")->getSFVec3f()[2];
}

WheelContacts wheelContacts(const WheelNodes& wheels, double floor_z_m) {
    WheelContacts contacts{};
    for (std::size_t leg = 0; leg < wheels.size(); ++leg) {
        int count = 0;
        const auto* points = wheels[leg]->getContactPoints(false, &count);
        for (int i = 0; i < count; ++i) {
            const double height = points[i].point[2] - floor_z_m;
            if (height >= -0.020 && height <= 0.002) {
                contacts[leg] = true;
                break;
            }
        }
    }
    return contacts;
}

double wheelClearance(const webots::Node& wheel, double floor_z_m) {
    // 圆柱局部 Y 轴在世界 Z 上的投影，决定半径与半宽对最低点的贡献。
    const double axle_z = std::clamp(std::abs(wheel.getOrientation()[7]), 0.0, 1.0);
    return wheel.getPosition()[2] - floor_z_m
           - WHEEL_RADIUS_M * std::sqrt(1.0 - axle_z * axle_z)
           - 0.5 * WHEEL_WIDTH_M * axle_z;
}

}  // namespace wl
