cmake_minimum_required(VERSION 3.8)
project(saws_teleop_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_executable(cartesian_teleop src/saws_ur5_cartesian_control.cpp)
ament_target_dependencies(cartesian_teleop rclcpp joy sensor_msgs geometry_msgs tf2_ros)

add_executable(joint_teleop src/saws_ur5_joint_control.cpp)
ament_target_dependencies(joint_teleop rclcpp joy sensor_msgs)

add_executable(gripper_teleop src/saws_gripper_control.cpp)
ament_target_dependencies(gripper_teleop rclcpp joy sensor_msgs control_msgs rclcpp_action)

install(TARGETS
  cartesian_teleop
  joint_teleop
  gripper_teleop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
