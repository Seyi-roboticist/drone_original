#ifndef X4_HARDWARE_INTERFACE_HPP_
#define X4_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <vector>
#include <limits>
#include <iostream>

class X4HardwareInterface : public hardware_interface::SystemInterface
{
public:
  X4HardwareInterface();
  ~X4HardwareInterface() override;

  // Called during startup for hardware initialization.
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // For this drone we assume that each joint (i.e. propeller) is controlled using velocity.
  std::vector<double> hw_commands_velocity;
  std::vector<double> hw_states_velocity;
};

#endif  
