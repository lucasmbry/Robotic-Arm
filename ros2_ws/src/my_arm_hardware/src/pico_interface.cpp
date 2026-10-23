#include "my_arm_hardware/pico_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace my_arm_hardware
{

namespace
{
constexpr size_t kNumJoints = 6;
constexpr double kResendEpsilon = 1e-5;  // rad; don't spam the Pico with identical packets
rclcpp::Logger logger() { return rclcpp::get_logger("PicoInterface"); }
}  // namespace

hardware_interface::CallbackReturn PicoInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(
      logger(), "Expected %zu joints in ros2_control description, got %zu",
      kNumJoints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional override from URDF.ros2_control.xacro:
  //   <param name="serial_port">/dev/ttyACM0</param>
  auto it = info_.hardware_parameters.find("serial_port");
  if (it != info_.hardware_parameters.end()) {
    serial_port_ = it->second;
  }

  hw_states_.assign(kNumJoints, 0.0);
  hw_commands_.assign(kNumJoints, 0.0);
  last_sent_commands_.assign(kNumJoints, 1e9);  // force first packet to send

  // Packet order == joint order in the ros2_control xacro (1st axis -> 6th axis).
  for (size_t i = 0; i < kNumJoints; ++i) {
    RCLCPP_INFO(logger(), "Packet slot %zu -> joint '%s'", i + 1, info_.joints[i].name.c_str());
  }

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
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ == -1) {
    RCLCPP_ERROR(
      logger(), "Unable to open serial port %s: %s", serial_port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios options;
  tcgetattr(serial_fd_, &options);
  cfmakeraw(&options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cc[VMIN] = 0;   // non-blocking reads
  options.c_cc[VTIME] = 0;
  tcsetattr(serial_fd_, TCSANOW, &options);
  tcflush(serial_fd_, TCIOFLUSH);

  // The Pico treats its power-on position as zero, so start from zero here too.
  for (size_t i = 0; i < kNumJoints; ++i) {
    hw_commands_[i] = hw_states_[i] = 0.0;
    last_sent_commands_[i] = 1e9;
  }
  rx_line_.clear();

  RCLCPP_INFO(logger(), "Activated Pico serial connection on %s.", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PicoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ != -1) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PicoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No encoder feedback (open-loop steppers): report the commanded position as the state.
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    hw_states_[i] = hw_commands_[i];
  }

  if (serial_fd_ == -1) {
    return hardware_interface::return_type::OK;
  }

  // Drain Pico logs, print complete lines only.
  char rx_buffer[256];
  ssize_t bytes_read;
  while ((bytes_read = ::read(serial_fd_, rx_buffer, sizeof(rx_buffer))) > 0) {
    rx_line_.append(rx_buffer, static_cast<size_t>(bytes_read));
  }

  size_t pos;
  while ((pos = rx_line_.find('\n')) != std::string::npos) {
    std::string line = rx_line_.substr(0, pos);
    rx_line_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!line.empty()) {
      RCLCPP_INFO(logger(), "[PICO]: %s", line.c_str());
    }
  }
  if (rx_line_.size() > 1024) rx_line_.clear();  // guard against runaway garbage

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PicoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ == -1) {
    return hardware_interface::return_type::OK;
  }

  // Only send when something actually changed.
  bool changed = false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(hw_commands_[i])) {
      return hardware_interface::return_type::OK;  // never send NaN to the Pico
    }
    if (std::fabs(hw_commands_[i] - last_sent_commands_[i]) > kResendEpsilon) {
      changed = true;
    }
  }
  if (!changed) {
    return hardware_interface::return_type::OK;
  }

  // Joint radians, in xacro joint order. The Pico applies microstepping and gear ratio itself.
  char buffer[160];
  int len = std::snprintf(
    buffer, sizeof(buffer), "<%.5f,%.5f,%.5f,%.5f,%.5f,%.5f>\n",
    hw_commands_[0], hw_commands_[1], hw_commands_[2],
    hw_commands_[3], hw_commands_[4], hw_commands_[5]);

  if (len > 0) {
    ssize_t written = ::write(serial_fd_, buffer, static_cast<size_t>(len));
    if (written == len) {
      last_sent_commands_ = hw_commands_;
    } else if (written < 0 && errno != EAGAIN) {
      RCLCPP_WARN_THROTTLE(
        logger(), *rclcpp::Clock::make_shared(), 2000, "Serial write failed: %s", std::strerror(errno));
    }
    // On a short write or EAGAIN, last_sent_commands_ stays stale so the next cycle retries.
  }

  return hardware_interface::return_type::OK;
}

}  // namespace my_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_arm_hardware::PicoInterface, hardware_interface::SystemInterface)