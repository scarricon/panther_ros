#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DATA_CONVERTERS_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DATA_CONVERTERS_HPP_

#include <atomic>
#include <condition_variable>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/script_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>

#include <panther_hardware_interfaces/roboteq_driver.hpp>

namespace panther_hardware_interfaces
{
struct DrivetrainSettings
{
  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
};

// TODO: unit tests

class MotorState
{
public:
  MotorState(DrivetrainSettings drivetrain_settings);

  void SetData(RoboteqMotorState fb) { last_state_ = fb; };

  float GetPosition() const { return last_state_.pos * roboteq_pos_feedback_to_radians_; }
  float GetVelocity() const
  {
    return last_state_.vel * roboteq_vel_feedback_to_radians_per_second_;
  }
  float GetTorque() const
  {
    return last_state_.current * roboteq_current_feedback_to_newton_meters_;
  }

private:
  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  RoboteqMotorState last_state_;
};

class RoboteqCommandConverter
{
public:
  RoboteqCommandConverter(DrivetrainSettings drivetrain_settings);
  int32_t Convert(double cmd) const { return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_); }

private:
  inline int32_t LimitCmd(int32_t cmd) const
  {
    return std::clamp(cmd, -max_roboteq_cmd_value_, max_roboteq_cmd_value_);
  }

  float radians_per_second_to_roboteq_cmd_;
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
};

// TODO restructure panther_msgs
class FaultFlag
{
public:
  FaultFlag() {}

  panther_msgs::msg::FaultFlag GetMessage() const;
  void SetData(uint8_t fault_flags, bool can_error)
  {
    fault_flags_ = fault_flags;
    can_error_ = can_error;
  }

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError() const { return (fault_flags_ & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;
  uint8_t fault_flags_ = 0b00000000;
  bool can_error_ = false;

  // TODO
  std::vector<std::string> driver_fault_flags_ = {
    "overheat",       "overvoltage",
    "undervoltage",   "short_circuit",
    "emergency_stop", "motor_or_sensor_setup_fault",
    "mosfet_failure", "default_config_loaded_at_startup",
  };
};

class ScriptFlag
{
public:
  ScriptFlag() {}

  panther_msgs::msg::ScriptFlag GetMessage() const;
  void SetData(uint8_t script_flags) { script_flags_ = script_flags; }

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError() const { return (script_flags_ & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;
  uint8_t script_flags_ = 0b00000000;

  // TODO
  std::vector<std::string> driver_script_flags_ = {
    "loop_error",
    "encoder_disconected",
    "amp_limiter",
  };
};

class RuntimeError
{
public:
  RuntimeError() {}

  panther_msgs::msg::RuntimeError GetMessage() const;
  void SetData(uint8_t runtime_errors_flags) { runtime_errors_flags_ = runtime_errors_flags; }

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError() const { return (runtime_errors_flags_ & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;
  uint8_t runtime_errors_flags_ = 0b00000000;

  // TODO
  std::vector<std::string> driver_runtime_errors_ = {
    "amps_limit_active",
    "motor_stall",
    "loop_error",
    "safety_stop_active",
    "forward_limit_triggered",
    "reverse_limit_triggered",
    "amps_trigger_activated",
  };
};

class DriverState
{
public:
  DriverState() {}

  void SetTemperature(int16_t temp) { last_temp_ = temp; };
  void SetVoltage(uint16_t voltage) { last_voltage_ = voltage; };
  void SetBatAmps1(int16_t bat_amps_1) { last_bat_amps_1_ = bat_amps_1; };
  void SetBatAmps2(int16_t bat_amps_2) { last_bat_amps_2_ = bat_amps_2; };

  float GetTemperature() const { return last_temp_; }
  float GetVoltage() const { return last_voltage_ / 10; }
  float GetCurrent() const { return last_bat_amps_1_ / 10.0 + last_bat_amps_2_ / 10.0; }

private:
  int16_t last_temp_;
  uint16_t last_voltage_;
  int16_t last_bat_amps_1_;
  int16_t last_bat_amps_2_;
};

class RoboteqData
{
public:
  RoboteqData(DrivetrainSettings drivetrain_settings)
  : left_state_(drivetrain_settings), right_state_(drivetrain_settings)
  {
    left_runtime_error_.SetSurpressedFlags(suppressed_runtime_errors_);
    right_runtime_error_.SetSurpressedFlags(suppressed_runtime_errors_);
  }

  void SetMotorStates(RoboteqMotorState left_state, RoboteqMotorState right_state, bool old_data)
  {
    left_state_.SetData(left_state);
    right_state_.SetData(right_state);
    old_data_ = old_data;
  }

  void SetFlags(
    uint8_t fault_flags, uint8_t script_flags, uint8_t left_runtime_errors_flags,
    uint8_t right_runtime_errors_flags, bool can_error)
  {
    fault_flags_.SetData(fault_flags, can_error);
    script_flags_.SetData(script_flags);
    left_runtime_error_.SetData(left_runtime_errors_flags);
    right_runtime_error_.SetData(right_runtime_errors_flags);
  }

  void SetTemperature(int16_t temp) { driver_state_.SetTemperature(temp); };
  void SetVoltage(uint16_t voltage) { driver_state_.SetVoltage(voltage); };
  void SetBatAmps1(int16_t bat_amps_1) { driver_state_.SetBatAmps1(bat_amps_1); };
  void SetBatAmps2(int16_t bat_amps_2) { driver_state_.SetBatAmps2(bat_amps_2); };

  bool IsError() const
  {
    return fault_flags_.IsError() || script_flags_.IsError() || left_runtime_error_.IsError() ||
           right_runtime_error_.IsError() || old_data_;
  }

  const MotorState & GetLeftMotorState() const { return left_state_; }
  const MotorState & GetRightMotorState() const { return right_state_; }
  const DriverState & GetDriverState() const { return driver_state_; }

  const FaultFlag & GetFaultFlag() const { return fault_flags_; }
  const ScriptFlag & GetScriptFlag() const { return script_flags_; }
  const RuntimeError & GetLeftRuntimeError() const { return left_runtime_error_; }
  const RuntimeError & GetRightRuntimeError() const { return right_runtime_error_; }

private:
  MotorState left_state_;
  MotorState right_state_;

  DriverState driver_state_;

  FaultFlag fault_flags_;
  ScriptFlag script_flags_;
  RuntimeError left_runtime_error_;
  RuntimeError right_runtime_error_;

  bool old_data_ = false;

  // TODO: to parameter

  // Suppress flags:
  // safety_stop_active
  // amps_limit_active
  uint8_t suppressed_runtime_errors_ = 0b11110110;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_