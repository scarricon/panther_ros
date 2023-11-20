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

#include <mock_roboteq.hpp>

void RoboteqSlave::SetPosition(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][1] = value;
  } else if (channel == 2) {
    (*this)[0x2106][2] = value;
  }
}

void RoboteqSlave::SetVelocity(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][3] = value;
  } else if (channel == 2) {
    (*this)[0x2106][4] = value;
  }
}

void RoboteqSlave::SetCurrent(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][5] = value;
  } else if (channel == 2) {
    (*this)[0x2106][6] = value;
  }
}

void RoboteqSlave::ClearErrorFlags()
{
  (*this)[0x2106][7] = 0;
  (*this)[0x2106][8] = 0;
}

void RoboteqSlave::InitializeValues()
{
  SetTemperature(0);
  SetVoltage(0);
  SetBatAmps1(0);
  SetBatAmps2(0);

  SetPosition(1, 0);
  SetPosition(2, 0);
  SetVelocity(1, 0);
  SetVelocity(2, 0);
  SetCurrent(1, 0);
  SetCurrent(2, 0);

  ClearErrorFlags();
};

void RoboteqSlave::StartPublishing(std::chrono::milliseconds period)
{
  pdo_publishing_thread_ = std::thread([this, period]() {
    auto next = std::chrono::steady_clock::now();
    while (!stop_publishing_) {
      next += period;
      // std::cout << "Publishing PDO" << std::endl;
      TriggerPDOPublish();
      std::this_thread::sleep_until(next);
    }
  });
}

void RoboteqSlave::StopPublishing()
{
  stop_publishing_.store(true);
  // TODO: joinable
  if (pdo_publishing_thread_.joinable()) {
    pdo_publishing_thread_.join();
  }
}

void RoboteqSlave::TriggerPDOPublish()
{
  // Every PDO holds two values - it is enough to send event to just one and both will be sent
  this->WriteEvent(0x2106, 1);
  this->WriteEvent(0x2106, 3);
  this->WriteEvent(0x2106, 5);
  this->WriteEvent(0x2106, 7);
}

void RoboteqSlave::SetDriverFaultFlag(DriverFaultFlags flag)
{
  int32_t current_data = (*this)[0x2106][7];
  current_data |= (0b00000001 << uint8_t(flag));
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverScriptFlag(DriverScriptFlags flag)
{
  int32_t current_data = (*this)[0x2106][7];
  current_data |= int32_t(0b00000001 << uint8_t(flag)) << 2 * 8;
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverRuntimeError(uint8_t channel, DriverRuntimeErrors flag)
{
  int32_t current_data = (*this)[0x2106][8];
  current_data |= int32_t(0b00000001 << uint8_t(flag)) << channel * 8;
  (*this)[0x2106][8] = current_data;
}

void RoboteqMock::Start(std::chrono::milliseconds pdo_period)
{
  canopen_communication_started_.store(false);
  ctx_ = std::make_shared<lely::io::Context>();

  canopen_communication_thread_ = std::thread([this, pdo_period]() {
    std::string slave_eds_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                   "panther_hardware_interfaces")) /
                                 "config" / "roboteq_motor_controllers_v80_21.eds";
    std::string slave1_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_1.bin";

    std::string slave2_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_2.bin";

    lely::io::IoGuard io_guard;
    lely::io::Poll poll(*ctx_);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    lely::io::CanController ctrl("panther_can");

    lely::io::CanChannel chan1(poll, exec);
    chan1.open(ctrl);
    lely::io::Timer timer1(poll, exec, CLOCK_MONOTONIC);
    front_driver_ = std::make_unique<RoboteqSlave>(
      timer1, chan1, slave_eds_path, slave1_eds_bin_path, 1);

    lely::io::CanChannel chan2(poll, exec);
    chan2.open(ctrl);
    lely::io::Timer timer2(poll, exec, CLOCK_MONOTONIC);
    rear_driver_ = std::make_unique<RoboteqSlave>(
      timer2, chan2, slave_eds_path, slave2_eds_bin_path, 2);

    front_driver_->Reset();
    rear_driver_->Reset();
    front_driver_->InitializeValues();
    rear_driver_->InitializeValues();
    front_driver_->StartPublishing(pdo_period);
    rear_driver_->StartPublishing(pdo_period);

    {
      std::lock_guard lk(canopen_communication_started_mtx_);
      canopen_communication_started_.store(true);
    }
    canopen_communication_started_cond_.notify_all();

    loop.run();

    front_driver_->StopPublishing();
    rear_driver_->StopPublishing();
  });

  if (!canopen_communication_started_.load()) {
    std::unique_lock lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait(lck);
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }
}

void RoboteqMock::Stop()
{
  ctx_->shutdown();
  canopen_communication_thread_.join();

  front_driver_.reset();
  rear_driver_.reset();

  canopen_communication_started_.store(false);
}

// int main()
// {
//   RoboteqMock roboteq_mock;
//   roboteq_mock.Start();
//   std::this_thread::sleep_for(std::chrono::seconds(30));
//   roboteq_mock.Stop();
//   return 0;
// }
