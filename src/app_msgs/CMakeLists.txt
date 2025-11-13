cmake_minimum_required(VERSION 3.8)
project(app_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GoTarget.srv"
    "msg/MotionType.msg"
    "srv/ArmLift.srv"
    "srv/ArmRotate.srv"
    "srv/ExploreBuild.srv"
    "srv/GoCharge.srv"
    "srv/SlamCommand.srv"
    "srv/CloudCommand.srv"
    "srv/SemanticMap.srv"
    "srv/SearchSomething.srv"
    "srv/StorageItem.srv"
    "srv/SaveImages.srv"
    "srv/RightArmDetect.srv"
    "action/GoCharge.action"
    "action/ExploreBuild.action"
    "action/AutomaticScraping.action"
    "action/SemanticNavigation.action"
    "action/GoTarget.action"  
    "msg/SearchSomething.msg"
    "msg/MappingOption.msg"
    "msg/MapOption.msg"
    "msg/MapInfo.msg"
    "msg/PncMessage.msg"
    "msg/VirtualWalls.msg"
    "msg/Semantic.msg"
    "msg/TaskMessage.msg"
    "msg/ObjectBox.msg"
    "msg/ObjectSemantic.msg"
    "msg/RoomSemantic.msg"
    "msg/NavigationOption.msg"
    "msg/ForbiddenZone.msg"
    "msg/MappingStatus.msg"
    "msg/VirtualWall.msg"
    "msg/AudioPlay.msg"
    "msg/MapMask.msg"
    "msg/ModifiedMap.msg"  
    DEPENDENCIES geometry_msgs
    DEPENDENCIES nav_msgs
    DEPENDENCIES std_msgs
  )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

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

