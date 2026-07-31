cmake_minimum_required(VERSION 3.8)
project(my_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Build the C++ file into a shared library (.so)
add_library(${PROJECT_NAME} SHARED src/pico_interface.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface pluginlib)

# Export the XML file we just created so ROS can find the plugin
pluginlib_export_plugin_description_file(hardware_interface my_arm_hardware.xml)

# Install rules (where to put the built files)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
