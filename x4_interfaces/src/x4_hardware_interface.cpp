#include "x4_interfaces/x4_hardware_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

X4HardwareInterface::X4HardwareInterface()
{
  std::cout << "X4HardwareInterface::X4HardwareInterface" << std::endl;
}

X4HardwareInterface::~X4HardwareInterface()
{
  std::cout << "X4HardwareInterface::~X4HardwareInterface" << std::endl;
}

hardware_interface::CallbackReturn X4HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  std::cout << "X4HardwareInterface::on_init" << std::endl;
  
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // For our drone we assume that each joint (propeller) uses a velocity interface.
  // Resize the state and command vectors accordingly.
  hw_states_velocity.resize(info.joints.size(), 0.0);
  hw_commands_velocity.resize(info.joints.size(), 0.0);

  for (const auto & joint : info.joints)
  {
    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        std::cout << joint.name << " has velocity command interface" << std::endl;
      }
    }
    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        std::cout << joint.name << " has velocity state interface" << std::endl;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> X4HardwareInterface::export_state_interfaces()
{
  std::cout << "X4HardwareInterface::export_state_interfaces" << std::endl;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocity[i]
      )
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> X4HardwareInterface::export_command_interfaces()
{
  std::cout << "X4HardwareInterface::export_command_interfaces" << std::endl;
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocity[i]
      )
    );
  }

  return command_interfaces;
}

hardware_interface::return_type X4HardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "X4HardwareInterface::read" << std::endl;
  // For demonstration purposes, assume the drone perfectly tracks the commanded velocity.
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    hw_states_velocity[i] = hw_commands_velocity[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type X4HardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "X4HardwareInterface::write" << std::endl;
  // Here you would normally send the command values to your drone's actuators.
  // For now, we simply print the commands.
  for (std::size_t i = 0; i < hw_commands_velocity.size(); ++i)
  {
    std::cout << "Command for " << info_.joints[i].name << ": " << hw_commands_velocity[i] << std::endl;
  }
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(X4HardwareInterface, hardware_interface::SystemInterface)
