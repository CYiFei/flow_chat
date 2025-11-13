cmake_minimum_required(VERSION 3.8)
project(serial_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyPositionMove.msg"
  "msg/ChargeInplace.msg"
  "msg/ChassisAutoMove.msg"
  "msg/ChassisFeedback.msg"
  "msg/DataCollection.msg"
  "msg/DriverStatus.msg"
  "msg/GimbleControl.msg"
  "msg/GimbleFeedback.msg"
  "msg/GimbleSetSpeed.msg"
  "msg/InfradeGuid.msg"
  "msg/LiftMoveCtrl.msg"
  "msg/RollingMoveCtrl.msg"
  "msg/RoboticArm.msg"
  "msg/RemoteControl.msg"
  "msg/Joint.msg"
  "msg/DualArmAutoMove.msg"
  "msg/DualArmStatus.msg"
  "msg/GripperAutoMove.msg"
  "msg/TwoFingerGripper.msg"
  "srv/ChargeControl.srv"
  "srv/ChassisEnable.srv"
  "srv/ChassisDisable.srv"
  "srv/ClearMotorError.srv"
  "srv/EmergStop.srv"
  "srv/GetRgbMode.srv"
  "srv/OpenCloseJoint.srv"
  "srv/PowerControl.srv"
  "srv/Restart32.srv"
  "srv/RestartCharge32.srv"
  "srv/SetMotorMode.srv"
  "srv/SetMotorPosition.srv"
  "srv/SetMotorZero.srv"
  "srv/SwitchRgbMode.srv"
  
  
  DEPENDENCIES std_msgs geometry_msgs
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
