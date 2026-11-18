#pragma once

#include "config.hpp"
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

namespace wl {

// 两种步态共用同一水平地面基准。轮节点由 Webots 管理。
using WheelNodes = std::array<webots::Node*, LEG_COUNT>;
using WheelContacts = std::array<bool, LEG_COUNT>;

double flatFloorHeight(webots::Supervisor& robot);
WheelContacts wheelContacts(const WheelNodes& wheels, double floor_z_m);
double wheelClearance(const webots::Node& wheel, double floor_z_m);

}  // namespace wl
