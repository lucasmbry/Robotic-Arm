#ifndef MY_ARM_HARDWARE__PICO_INTERFACE_HPP_
#define MY_ARM_HARDWARE__PICO_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>

namespace my_arm_hardware
{
class PicoInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication (File Descriptor)
  int serial_fd_; 
  
  // Data arrays matching the number of joints
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  
  // Tracking steps for the delta calculations
  std::vector<long> last_sent_steps_;
  std::vector<double> steps_per_rad_;
};

}  // namespace my_arm_hardware

#endif  // MY_ARM_HARDWARE__PICO_INTERFACE_HPP_
