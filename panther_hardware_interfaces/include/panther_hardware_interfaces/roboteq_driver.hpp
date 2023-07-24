#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

namespace panther_hardware_interfaces
{

enum class RoboteqMode { POSITION = 0, VELOCITY = 1, TORQUE = 2 };
struct RoboteqChannelFeedback
{
  int pos;
  int vel;
  int current;
  uint8_t runtime_stat_flag;
};

struct RoboteqMotorsFeedback
{
  RoboteqChannelFeedback motor_1;
  RoboteqChannelFeedback motor_2;
  uint8_t fault_flags;
  uint8_t script_flags;
  timespec timestamp;
};

struct RoboteqDriverFeedback
{
  float temp;
  bool temp_error;

  float voltage;
  bool voltage_error;

  float bat_amps_1;
  bool bat_amps_1_error;

  float bat_amps_2;
  bool bat_amps_2_error;
};

// All ids and sub ids were read directly from eds file
// lely canopen doesn't have option to parse them based on the ParameterName
// additionally between version v60 and v80 ParameterName changed:
// ParameterName=Cmd_ESTOP (old)
// ParameterName=Cmd_ESTOP Emergency Shutdown (new)
// which would require looking for substring
// additionally as it is visible parameter names changed, but ids stayed the same, so
// it will be better to just use ids directly

class RoboteqDriver : public lely::canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;

  RoboteqDriverFeedback ReadRoboteqDriverFeedback();
  RoboteqMotorsFeedback ReadRoboteqMotorsFeedback();
  void SendRoboteqCmd(int32_t channel_1_cmd, int32_t channel_2_cmd);

  void ResetRoboteqScript();
  void SetVelocityMode();
  void TurnOnEstop();
  void TurnOffEstop();

  bool wait_for_boot();
  bool is_booted() { return booted.load(); }
  bool get_can_error() { return can_error.load(); }
  bool Boot();

private:
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
  int32_t LimitCmd(int32_t cmd);
  uint8_t GetByte(uint32_t data, uint8_t byte_no);

  std::atomic<bool> booted;
  std::condition_variable boot_cond;
  std::mutex boot_mtx;
  std::string boot_what;

  std::mutex can_error_mtx;
  std::atomic<bool> can_error;
  lely::io::CanError can_error_code;

  std::mutex rpdo_timestamp_mtx_;

  // TODO
  void OnState(lely::canopen::NmtState state) noexcept override;

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  // void OnTpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  timespec last_rpdo_write_timestamp_;

  // emcy - emergency - I don't think that it is used by roboteq - haven't found any information about it
  // while ros2_canopen has ability to read it, I didn't see any attempts to handle it
  // void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  void OnCanError(lely::io::CanError error) noexcept override;
  // virtual void OnConfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // virtual void OnDeconfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // void
  // Error()
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_