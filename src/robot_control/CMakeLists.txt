cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(service_msgs REQUIRED) 
#find_package(type_description_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Build the library
add_library(cli_manual_control src/cli_manual_control.cpp)
target_include_directories(cli_manual_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_control>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cli_manual_control rclcpp std_msgs )

add_library(serial_communication src/serial_communication.cpp)
target_include_directories(serial_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_control>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serial_communication rclcpp std_msgs )

# Build the executable
add_executable(motor_control_node src/robot_control.cpp)
target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_control_node rclcpp std_msgs )
target_link_libraries(motor_control_node cli_manual_control)
target_link_libraries(motor_control_node serial_communication)
# Install targets
install(TARGETS
  cli_manual_control
  serial_communication
  motor_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()