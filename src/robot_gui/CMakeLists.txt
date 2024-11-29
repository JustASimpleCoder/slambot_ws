cmake_minimum_required(VERSION 3.8)
project(robot_gui)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)


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

add_executable(robot_gui_node src/robot_gui.cpp)

target_link_libraries(robot_gui_node OpenGL::GL glfw) 

ament_target_dependencies(robot_gui_node rclcpp std_msgs)

install(TARGETS
  robot_gui_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()