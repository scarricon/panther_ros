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
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

#include <iostream>

class TestRoboteqDriver : public ::testing::Test
{
public:
  TestRoboteqDriver()
  {
    canopen_controller_ = std::make_unique<panther_hardware_interfaces::CanOpenController>(
      panther_hardware_interfaces_test::kCanopenSettings);

    roboteq_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqMock>();
    roboteq_mock_->Start(std::chrono::milliseconds(100));
    canopen_controller_->Initialize();
  }

  ~TestRoboteqDriver()
  {
    canopen_controller_->Deinitialize();
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }

  std::unique_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;

  std::unique_ptr<panther_hardware_interfaces::CanOpenController> canopen_controller_;
};

// These tests are related to canopen_controller tests, where boot should be already tested

TEST_F(TestRoboteqDriver, test_read_temperature)
{
  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);

  ASSERT_EQ(canopen_controller_->GetFrontDriver()->ReadTemperature(), f_temp);
  ASSERT_EQ(canopen_controller_->GetRearDriver()->ReadTemperature(), r_temp);
}

TEST_F(TestRoboteqDriver, test_read_voltage)
{
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;

  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);

  ASSERT_EQ(canopen_controller_->GetFrontDriver()->ReadVoltage(), f_volt);
  ASSERT_EQ(canopen_controller_->GetRearDriver()->ReadVoltage(), r_volt);
}

TEST_F(TestRoboteqDriver, test_read_bat_amps1)
{
  const std::int16_t f_bat_amps_1 = 10;
  const std::int16_t r_bat_amps_1 = 30;

  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->rear_driver_->SetBatAmps1(r_bat_amps_1);

  ASSERT_EQ(canopen_controller_->GetFrontDriver()->ReadBatAmps1(), f_bat_amps_1);
  ASSERT_EQ(canopen_controller_->GetRearDriver()->ReadBatAmps1(), r_bat_amps_1);
}

TEST_F(TestRoboteqDriver, test_read_bat_amps2)
{
  const std::int16_t f_bat_amps_2 = 30;
  const std::int16_t r_bat_amps_2 = 40;

  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);
  roboteq_mock_->rear_driver_->SetBatAmps2(r_bat_amps_2);

  ASSERT_EQ(canopen_controller_->GetFrontDriver()->ReadBatAmps2(), f_bat_amps_2);
  ASSERT_EQ(canopen_controller_->GetRearDriver()->ReadBatAmps2(), r_bat_amps_2);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_driver_feedback_values)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverFaultFlags;
  using panther_hardware_interfaces_test::DriverRuntimeErrors;
  using panther_hardware_interfaces_test::DriverScriptFlags;

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

  panther_hardware_interfaces::RoboteqDriverFeedback f_fb =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverFeedback();
  panther_hardware_interfaces::RoboteqDriverFeedback r_fb =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverFeedback();

  ASSERT_EQ(f_fb.motor_2.pos, fl_pos);
  ASSERT_EQ(f_fb.motor_2.vel, fl_vel);
  ASSERT_EQ(f_fb.motor_2.current, fl_current);

  ASSERT_EQ(f_fb.motor_1.pos, fr_pos);
  ASSERT_EQ(f_fb.motor_1.vel, fr_vel);
  ASSERT_EQ(f_fb.motor_1.current, fr_current);

  ASSERT_EQ(r_fb.motor_2.pos, rl_pos);
  ASSERT_EQ(r_fb.motor_2.vel, rl_vel);
  ASSERT_EQ(r_fb.motor_2.current, rl_current);

  ASSERT_EQ(r_fb.motor_1.pos, rr_pos);
  ASSERT_EQ(r_fb.motor_1.vel, rr_vel);
  ASSERT_EQ(r_fb.motor_1.current, rr_current);

  ASSERT_EQ(f_fb.fault_flags, 0b00000001);
  ASSERT_EQ(f_fb.script_flags, 0b00000010);
  ASSERT_EQ(f_fb.runtime_stat_flag_motor_1, 0b00000100);
  ASSERT_EQ(f_fb.runtime_stat_flag_motor_2, 0b00001000);

  ASSERT_EQ(r_fb.fault_flags, 0b00000010);
  ASSERT_EQ(r_fb.script_flags, 0b00000100);
  ASSERT_EQ(r_fb.runtime_stat_flag_motor_1, 0b00010000);
  ASSERT_EQ(r_fb.runtime_stat_flag_motor_2, 0b00100000);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_driver_feedback_timestamp)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  panther_hardware_interfaces::RoboteqDriverFeedback f_fb1 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverFeedback();
  panther_hardware_interfaces::RoboteqDriverFeedback r_fb1 =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverFeedback();

  // based on publishing frequency in the Roboteq mock (100)
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  panther_hardware_interfaces::RoboteqDriverFeedback f_fb2 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverFeedback();
  panther_hardware_interfaces::RoboteqDriverFeedback r_fb2 =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverFeedback();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  ASSERT_LE(
    lely::util::from_timespec(f_fb2.timestamp) - lely::util::from_timespec(f_fb1.timestamp),
    std::chrono::milliseconds(102));
  ASSERT_LE(
    lely::util::from_timespec(r_fb2.timestamp) - lely::util::from_timespec(r_fb1.timestamp),
    std::chrono::milliseconds(102));
}

TEST_F(TestRoboteqDriver, test_send_roboteq_cmd)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t fl_v = 10;
  const std::int32_t fr_v = 20;
  const std::int32_t rl_v = 30;
  const std::int32_t rr_v = 40;

  canopen_controller_->GetFrontDriver()->SendRoboteqCmdChannel1(fr_v);
  canopen_controller_->GetFrontDriver()->SendRoboteqCmdChannel2(fl_v);
  canopen_controller_->GetRearDriver()->SendRoboteqCmdChannel1(rr_v);
  canopen_controller_->GetRearDriver()->SendRoboteqCmdChannel2(rl_v);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), fl_v);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), fr_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), rl_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), rr_v);
}

TEST_F(TestRoboteqDriver, test_reset_roboteq_script)
{
  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  canopen_controller_->GetFrontDriver()->ResetRoboteqScript();
  canopen_controller_->GetRearDriver()->ResetRoboteqScript();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_turn_on_estop)
{
  roboteq_mock_->front_driver_->SetTurnOnEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEstop(23);

  canopen_controller_->GetFrontDriver()->TurnOnEstop();
  canopen_controller_->GetRearDriver()->TurnOnEstop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEstop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_off_estop)
{
  roboteq_mock_->front_driver_->SetTurnOffEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEstop(23);

  canopen_controller_->GetFrontDriver()->TurnOffEstop();
  canopen_controller_->GetRearDriver()->TurnOffEstop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEstop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_on_safety_stop_channel_1)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(67);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(21);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel1();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel1();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_on_safety_stop_channel_2)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel2();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel2();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, test_write_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<std::int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(
    canopen_controller_->GetFrontDriver()->SendRoboteqCmdChannel1(0), std::runtime_error);
}

TEST_F(TestRoboteqDriver, test_read_timeout)
{
  roboteq_mock_->front_driver_->SetOnReadWait<std::int8_t>(0x210F, 1, 100000);
  ASSERT_THROW(canopen_controller_->GetFrontDriver()->ReadTemperature(), std::runtime_error);
}

// OnCanError isn't tested, because it reacts to lower-level CAN errors (CRC), which are hard to
// simulate, but it would be nice to add it

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
