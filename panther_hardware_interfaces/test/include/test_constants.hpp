// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_
#define PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

namespace panther_hardware_interfaces_test
{

const panther_hardware_interfaces::CanOpenSettings kCanopenSettings{
  3, 1, 2, std::chrono::milliseconds(15), std::chrono::milliseconds(4)};

const panther_hardware_interfaces::DrivetrainSettings kDrivetrainSettings{
  0.11, 30.08, 0.75, 1600.0, 3600.0};

constexpr float kRadPerSecToRbtqCmd = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);
constexpr float kRbtqPosFbToRad = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
constexpr float kRbtqVelFbToRadPerSec = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
constexpr float kRbtqCurrentFbToNewtonMeters = (1. / 10.) * 0.11 * 30.08 * 0.75;

const std::string kPantherSystemName = "wheels";

const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="wheels" type="system">
)";

const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

const std::string kJointInterfaces =
  R"(<command_interface name="velocity" />
<state_interface name="position" />
<state_interface name="velocity" />
<state_interface name="effort" />
)";

const std::string kPluginName =
  R"(<plugin>panther_hardware_interfaces/PantherSystem</plugin>
)";

const std::map<std::string, std::string> kDefaultParamMap = {
  {"encoder_resolution", "1600"},
  {"gear_ratio", "30.08"},
  {"gearbox_efficiency", "0.75"},
  {"motor_torque_constant", "0.11"},
  {"max_rpm_motor_speed", "3600.0"},
  {"master_can_id", "3"},
  {"front_driver_can_id", "1"},
  {"rear_driver_can_id", "2"},
  {"sdo_operation_timeout", "4"},
  {"pdo_feedback_timeout", "15"},
  {"max_roboteq_initialization_attempts", "3"},
  {"max_roboteq_activation_attempts", "3"},
  {"max_safety_stop_attempts", "20"},
  {"max_write_sdo_errors_count", "2"},
  {"max_read_sdo_errors_count", "2"},
  {"max_read_pdo_errors_count", "1"},
};

const std::vector<std::string> kDefaultJoints = {
  "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"};

const std::string kMotorControllersStateTopic =
  "/panther_system_node/driver/motor_controllers_state";

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_
