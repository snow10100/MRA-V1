cmake_minimum_required(VERSION 3.8)
project(arm_6dof)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_driver REQUIRED)


add_library(
  arm_6dof
  SHARED
  src/arm_6dof.cpp
)
target_include_directories(
  arm_6dof
  PUBLIC
  include
)
ament_target_dependencies(
  arm_6dof
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  serial_driver
)
# prevent pluginlib from using boost
target_compile_definitions(arm_6dof PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface arm_6dof.xml)

install(
  TARGETS
  arm_6dof
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  arm_6dof
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  arm_6dof
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
