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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_

#include <cmath>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>

#include <roboteq_mock.hpp>

namespace panther_hardware_interfaces_test
{

class PantherSystemTestUtils
{
public:
  PantherSystemTestUtils() { default_panther_system_urdf_ = BuildUrdf(param_map_, joints_); }

  void SetState(const uint8_t state_id, const std::string & state_name);
  void ConfigurePantherSystem();
  void UnconfigurePantherSystem();
  void ActivatePantherSystem();
  void DeactivatePantherSystem();
  void ShutdownPantherSystem();
  void ConfigureActivatePantherSystem();
  void Start(std::string urdf);
  void Stop();
  std::string BuildUrdf(
    std::map<std::string, std::string> param_map, std::vector<std::string> joints);

  std::unique_ptr<RoboteqMock> roboteq_mock_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;

  // TODO
  double rad_per_sec_to_rbtq_cmd_ = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);
  double rbtq_pos_fb_to_rad_ = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
  double rbtq_vel_fb_to_rad_per_sec_ = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
  double rbtq_current_fb_to_newton_meters_ = (1. / 10.) * 0.11 * 30.08 * 0.75;

  const std::string panther_system_name_ = "wheels";

  const std::string urdf_header_ = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="wheels" type="system">
)";

  const std::string urdf_footer_ = R"(</ros2_control>
</robot>
)";

  const std::string joint_interfaces_ =
    R"(<command_interface name="velocity" />
<state_interface name="position" />
<state_interface name="velocity" />
<state_interface name="effort" />
)";

  std::string default_panther_system_urdf_;

  std::map<std::string, std::string> param_map_ = {
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

  std::vector<std::string> joints_ = {
    "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"};
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_
