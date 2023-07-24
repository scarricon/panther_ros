#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_

#include <thread>

#include <gpiod.hpp>

namespace panther_hardware_interfaces
{

// Placeholder GPIO class used for simple enabling, will be replaced by proper implementation in the future

enum class GPIOPins { VMOT_ON = 6, MOTOR_DRIVER_EN = 23, WATCHDOG = 14, E_STOP_RESET = 27 };

class GPIOController
{
public:
  GPIOController();
  ~GPIOController();

  void Start();

private:
  std::unique_ptr<gpiod::chip> chip_;
  std::unique_ptr<gpiod::line> vmot_;
  std::unique_ptr<gpiod::line> motor_driver_;
  std::unique_ptr<gpiod::line> e_stop_reset_;
  std::unique_ptr<gpiod::line> watchdog_;

  std::unique_ptr<std::thread> watchdog_thread_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_