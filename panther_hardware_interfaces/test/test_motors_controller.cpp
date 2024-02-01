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

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/motors_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

class TestMotorsControllerInitialization : public ::testing::Test
{
public:
  TestMotorsControllerInitialization()
  {
    motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
      panther_hardware_interfaces_test::kCANopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings);

    roboteq_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqMock>();
    roboteq_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
  }

  ~TestMotorsControllerInitialization()
  {
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }

  std::shared_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;
};

// These tests are related to canopen_controller tests, where boot should be already tested

TEST_F(TestMotorsControllerInitialization, test_initialize)
{
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_error_device_type)
{
  roboteq_mock_->front_driver_->SetOnReadWait<std::uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteq_mock_->front_driver_->SetOnReadWait<std::uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_error_vendor_id)
{
  roboteq_mock_->rear_driver_->SetOnReadWait<std::uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteq_mock_->rear_driver_->SetOnReadWait<std::uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_activate)
{
  using panther_hardware_interfaces_test::DriverChannel;

  motors_controller_->Initialize();

  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  ASSERT_NO_THROW(motors_controller_->Activate());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  motors_controller_->Deinitialize();
}

TEST_F(TestMotorsControllerInitialization, test_activate_sdo_timeout_reset)
{
  motors_controller_->Initialize();
  roboteq_mock_->front_driver_->SetOnWriteWait<std::uint8_t>(0x2018, 0, 100000);
  ASSERT_THROW(motors_controller_->Activate(), std::runtime_error);
  motors_controller_->Deinitialize();
}

class TestMotorsController : public TestMotorsControllerInitialization
{
public:
  TestMotorsController()
  {
    motors_controller_->Initialize();
    motors_controller_->Activate();
  }

  ~TestMotorsController() { motors_controller_->Deinitialize(); }
};

TEST_F(TestMotorsController, test_update_motors_states)
{
  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using panther_hardware_interfaces_test::kRbtqPosFbToRad;
  using panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  const std::int32_t fl_pos = 101;
  const std::int32_t fl_vel = 102;
  const std::int32_t fl_current = 103;
  const std::int32_t fr_pos = 201;
  const std::int32_t fr_vel = 202;
  const std::int32_t fr_current = 203;
  const std::int32_t rl_pos = 301;
  const std::int32_t rl_vel = 302;
  const std::int32_t rl_current = 303;
  const std::int32_t rr_pos = 401;
  const std::int32_t rr_vel = 402;
  const std::int32_t rr_current = 403;

  roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL2, fl_pos);
  roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL1, fr_pos);
  roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL2, rl_pos);
  roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL1, rr_pos);

  roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL2, fl_vel);
  roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL1, fr_vel);
  roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL2, rl_vel);
  roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL1, rr_vel);

  roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL2, fl_current);
  roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL1, fr_current);
  roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL2, rl_current);
  roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL1, rr_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  motors_controller_->UpdateMotorsStates();

  const auto & fl = motors_controller_->GetFrontData().GetLeftMotorState();
  const auto & fr = motors_controller_->GetFrontData().GetRightMotorState();
  const auto & rl = motors_controller_->GetRearData().GetLeftMotorState();
  const auto & rr = motors_controller_->GetRearData().GetRightMotorState();

  ASSERT_FLOAT_EQ(fl.GetPosition(), fl_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(fl.GetVelocity(), fl_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(fl.GetTorque(), fl_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(fr.GetPosition(), fr_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(fr.GetVelocity(), fr_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(fr.GetTorque(), fr_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(rl.GetPosition(), rl_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rl.GetVelocity(), rl_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rl.GetTorque(), rl_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(rr.GetPosition(), rr_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rr.GetVelocity(), rr_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rr.GetTorque(), rr_current * kRbtqCurrentFbToNewtonMeters);
}

TEST_F(TestMotorsController, test_update_motors_states_timestamps)
{
  motors_controller_->UpdateMotorsStates();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateMotorsStates();

  ASSERT_FALSE(motors_controller_->GetFrontData().IsMotorStatesDataTimedOut());
  ASSERT_FALSE(motors_controller_->GetRearData().IsMotorStatesDataTimedOut());
}

TEST(TestMotorsControllerOthers, test_update_motors_states_timeout)
{
  std::shared_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;

  motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
    panther_hardware_interfaces_test::kCANopenSettings,
    panther_hardware_interfaces_test::kDrivetrainSettings);

  roboteq_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqMock>();

  roboteq_mock_->Start(std::chrono::milliseconds(200), std::chrono::milliseconds(50));

  motors_controller_->Initialize();
  motors_controller_->Activate();

  motors_controller_->UpdateMotorsStates();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateMotorsStates();

  ASSERT_TRUE(motors_controller_->GetFrontData().IsMotorStatesDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetRearData().IsMotorStatesDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetFrontData().IsError());
  ASSERT_TRUE(motors_controller_->GetRearData().IsError());

  motors_controller_->Deinitialize();

  roboteq_mock_->Stop();
  roboteq_mock_.reset();
}

// Similar to test_roboteq_driver, can_error in update_system_feedback isn't tested, because it
// reacts to lower-level CAN errors (CRC), which are hard to simulate, but it would be nice to add
// it

TEST_F(TestMotorsController, test_update_driver_state)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverFaultFlags;
  using panther_hardware_interfaces_test::DriverRuntimeErrors;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;
  const std::int16_t f_heatsink_temp = 31;
  const std::int16_t r_heatsink_temp = 33;
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t r_battery_current_1 = 30;
  const std::int16_t f_battery_current_2 = 30;
  const std::int16_t r_battery_current_2 = 40;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);
  roboteq_mock_->front_driver_->SetHeatsinkTemperature(f_heatsink_temp);
  roboteq_mock_->rear_driver_->SetHeatsinkTemperature(r_heatsink_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);
  roboteq_mock_->front_driver_->SetBatteryCurrent1(f_battery_current_1);
  roboteq_mock_->rear_driver_->SetBatteryCurrent1(r_battery_current_1);
  roboteq_mock_->front_driver_->SetBatteryCurrent2(f_battery_current_2);
  roboteq_mock_->rear_driver_->SetBatteryCurrent2(r_battery_current_2);

  roboteq_mock_->front_driver_->SetDriverFaultFlag(DriverFaultFlags::OVERHEAT);
  roboteq_mock_->front_driver_->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);
  roboteq_mock_->front_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::LOOP_ERROR);
  roboteq_mock_->front_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::SAFETY_STOP_ACTIVE);

  roboteq_mock_->rear_driver_->SetDriverFaultFlag(DriverFaultFlags::OVERVOLTAGE);
  roboteq_mock_->rear_driver_->SetDriverScriptFlag(DriverScriptFlags::AMP_LIMITER);
  roboteq_mock_->rear_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::FORWARD_LIMIT_TRIGGERED);
  roboteq_mock_->rear_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::REVERSE_LIMIT_TRIGGERED);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  motors_controller_->UpdateDriversState();

  const auto & front = motors_controller_->GetFrontData();
  const auto & rear = motors_controller_->GetRearData();
  const auto & front_driver_state = motors_controller_->GetFrontData().GetDriverState();
  const auto & rear_driver_state = motors_controller_->GetRearData().GetDriverState();

  ASSERT_EQ(static_cast<std::int16_t>(front_driver_state.GetTemperature()), f_temp);
  ASSERT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetHeatsinkTemperature()), f_heatsink_temp);
  ASSERT_EQ(static_cast<std::uint16_t>(front_driver_state.GetVoltage() * 10.0), f_volt);
  ASSERT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetCurrent() * 10.0),
    f_battery_current_1 + f_battery_current_2);

  ASSERT_EQ(static_cast<std::int16_t>(rear_driver_state.GetTemperature()), r_temp);
  ASSERT_EQ(static_cast<std::int16_t>(rear_driver_state.GetHeatsinkTemperature()), r_heatsink_temp);
  ASSERT_EQ(static_cast<std::uint16_t>(rear_driver_state.GetVoltage() * 10.0), r_volt);
  ASSERT_EQ(
    static_cast<std::int16_t>(rear_driver_state.GetCurrent() * 10.0),
    r_battery_current_1 + r_battery_current_2);

  ASSERT_TRUE(front.GetFaultFlag().GetMessage().overheat);
  ASSERT_TRUE(front.GetScriptFlag().GetMessage().encoder_disconected);
  ASSERT_TRUE(front.GetRightRuntimeError().GetMessage().loop_error);
  ASSERT_TRUE(front.GetLeftRuntimeError().GetMessage().safety_stop_active);

  ASSERT_TRUE(rear.GetFaultFlag().GetMessage().overvoltage);
  ASSERT_TRUE(rear.GetScriptFlag().GetMessage().amp_limiter);
  ASSERT_TRUE(rear.GetRightRuntimeError().GetMessage().forward_limit_triggered);
  ASSERT_TRUE(rear.GetLeftRuntimeError().GetMessage().reverse_limit_triggered);
}

TEST_F(TestMotorsController, test_update_driver_state_timestamps)
{
  motors_controller_->UpdateDriversState();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateDriversState();

  ASSERT_FALSE(motors_controller_->GetFrontData().IsDriverStateDataTimedOut());
  ASSERT_FALSE(motors_controller_->GetRearData().IsDriverStateDataTimedOut());
}

TEST(TestMotorsControllerOthers, test_update_driver_state_timeout)
{
  std::shared_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;

  motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
    panther_hardware_interfaces_test::kCANopenSettings,
    panther_hardware_interfaces_test::kDrivetrainSettings);

  roboteq_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqMock>();

  roboteq_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(200));

  motors_controller_->Initialize();
  motors_controller_->Activate();

  motors_controller_->UpdateDriversState();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateDriversState();

  ASSERT_TRUE(motors_controller_->GetFrontData().IsDriverStateDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetRearData().IsDriverStateDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetFrontData().IsError());
  ASSERT_TRUE(motors_controller_->GetRearData().IsError());

  motors_controller_->Deinitialize();

  roboteq_mock_->Stop();
  roboteq_mock_.reset();
}

TEST_F(TestMotorsController, test_write_speed)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  ASSERT_NO_THROW(motors_controller_->SendSpeedCommands(fl_v, fr_v, rl_v, rr_v));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));
}

// Similar to test_roboteq_driver, can_error in write speed isn't tested, because it reacts to lower
// level CAN errors (CRC), which are hard to simulate, but it would be nice to add it

TEST_F(TestMotorsController, test_turn_on_e_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnEStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEStop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOnEStop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEStop(), 1);
}

TEST_F(TestMotorsController, test_turn_off_e_stop)
{
  roboteq_mock_->front_driver_->SetTurnOffEStop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEStop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOffEStop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEStop(), 1);
}

TEST_F(TestMotorsController, test_turn_on_e_stop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<std::uint8_t>(0x200C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnEStop(), std::runtime_error);
}

TEST_F(TestMotorsController, test_turn_off_e_stop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<std::uint8_t>(0x200D, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOffEStop(), std::runtime_error);
}

TEST_F(TestMotorsController, test_safety_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  bool front_driver_channel1_safety_stop = false;
  bool rear_driver_channel1_safety_stop = false;

  std::atomic_bool finish_test = false;

  // Check if first channel was set in the meantime - not sure how robust this test will be - as
  // safety stops for channel 1 and 2 are set just after one another, it is necessary to check value
  // of the current channel set frequently (and performance can vary on different machines)
  auto channel1_test_thread = std::thread([roboteq_mock = roboteq_mock_, &finish_test,
                                           &front_driver_channel1_safety_stop,
                                           &rear_driver_channel1_safety_stop]() {
    while (true) {
      if (
        front_driver_channel1_safety_stop == false &&
        roboteq_mock->front_driver_->GetTurnOnSafetyStop() == 1) {
        front_driver_channel1_safety_stop = true;
      }

      if (
        rear_driver_channel1_safety_stop == false &&
        roboteq_mock->rear_driver_->GetTurnOnSafetyStop() == 1) {
        rear_driver_channel1_safety_stop = true;
      }

      if (finish_test || (front_driver_channel1_safety_stop && rear_driver_channel1_safety_stop)) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });

  ASSERT_NO_THROW(motors_controller_->TurnOnSafetyStop());

  finish_test = true;
  channel1_test_thread.join();

  ASSERT_TRUE(front_driver_channel1_safety_stop);
  ASSERT_TRUE(rear_driver_channel1_safety_stop);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestMotorsController, test_safety_stop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<std::uint8_t>(0x202C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
