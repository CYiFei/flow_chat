cmake_minimum_required(VERSION 3.5)
project(arm_interfaces)
SET(CMAKE_BUILD_TYPE "Release")

option(BUILD_TESTING "" ON)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated -fPIC)
endif()


set(dependencies
  rclcpp
  ament_cmake
  rosidl_default_generators
  std_msgs
  builtin_interfaces
  geometry_msgs
)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmTask.action"
  "msg/AutonomyArmCommand.msg"
  "msg/StatusStream.msg"
  "msg/ArmStatus.msg"
  "msg/JointStatus.msg"
  "msg/MasterArmCommand.msg"
  "msg/PerceptionInformation.msg"
  "msg/PerceptionTarget.msg"
  "msg/NeckCommand.msg"
  "srv/LiftController.srv" 
  "srv/NeckTask.srv" 
  "srv/TrajFollowMode.srv" 
  "srv/EnableArmMode.srv"
  "srv/SetModeParam.srv"
  DEPENDENCIES ${dependencies}
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
