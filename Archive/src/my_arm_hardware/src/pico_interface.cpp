#include "my_arm_hardware/pico_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

namespace my_arm_hardware
{

hardware_interface::CallbackReturn PicoInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Allocate memory for our joints (Base, Shoulder, Elbow, Wrist = 4)
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  last_sent_steps_.resize(info_.joints.size(), 0);
  steps_per_rad_.resize(info_.joints.size(), 0.0);

  // Example: Read parameters passed from your URDF
  // ROS now just passes raw radians directly to the Pico!
  steps_per_rad_[0] = 1.0;
  steps_per_rad_[1] = 1.0;
  steps_per_rad_[2] = 1.0;
  steps_per_rad_[3] = 1.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PicoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PicoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn PicoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 1. OPEN SERIAL PORT TO PICO (Keeping O_NDELAY so ROS 2 doesn't block)
  serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("PicoInterface"), "Unable to open serial port.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2. CONFIGURE BAUD RATE (Upgraded to 115200)
  struct termios options;
  tcgetattr(serial_fd_, &options);
  cfsetispeed(&options, B115200); // <--- CHANGE THIS
  cfsetospeed(&options, B115200); // <--- CHANGE THIS
  options.c_cflag |= (CLOCAL | CREAD); 
  cfmakeraw(&options); 
  tcsetattr(serial_fd_, TCSANOW, &options);

  // 3. ZERO OUT STARTING POSITIONS
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    hw_commands_[i] = hw_states_[i] = 0.0;
    last_sent_steps_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("PicoInterface"), "Successfully activated Pico serial connection.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  close(serial_fd_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PicoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. THE "OPEN LOOP" LIE
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    hw_states_[i] = hw_commands_[i];
  }

  // 2. DRAIN AND PRINT THE PICO'S DEBUG LOGS
  char rx_buffer[256];
  // Non-blocking read of whatever the Pico just printed
  int bytes_read = ::read(serial_fd_, rx_buffer, sizeof(rx_buffer) - 1);
  
  if (bytes_read > 0) {
    rx_buffer[bytes_read] = '\0'; // Null-terminate
    
    // Convert to string and strip trailing newline for cleaner logs
    std::string msg(rx_buffer);
    if (!msg.empty() && msg.back() == '\n') msg.pop_back();
    if (!msg.empty() && msg.back() == '\r') msg.pop_back();
    
    // Print the Pico's internal state to your ROS terminal!
    RCLCPP_INFO(rclcpp::get_logger("PicoInterface"), "[PICO W]: %s", msg.c_str());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PicoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Declare the variables
  float base_rad = 0.0, shoulder_rad = 0.0, elbow_rad = 0.0, wrist_rad = 0.0;
  
  // 2. Map indices explicitly by name to guarantee order
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].name == "Revolute 7") base_rad = hw_commands_[i];
    else if (info_.joints[i].name == "Revolute 8") shoulder_rad = hw_commands_[i];
    else if (info_.joints[i].name == "Revolute 10") elbow_rad = hw_commands_[i];
    else if (info_.joints[i].name == "Revolute 11") wrist_rad = hw_commands_[i];
  }

  // 3. Format precisely to save bytes
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "<%.5f,%.5f,%.5f,%.5f>\n", base_rad, shoulder_rad, elbow_rad, wrist_rad);
  
  std::string serial_packet = buffer;

  // 4. Fire and forget (Non-blocking write)
  ::write(serial_fd_, serial_packet.c_str(), serial_packet.length());

  return hardware_interface::return_type::OK;
}

}  // namespace my_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_arm_hardware::PicoInterface, hardware_interface::SystemInterface)
