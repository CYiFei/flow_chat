cmake_minimum_required(VERSION 3.8)
project(chat_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(iflytek_msg REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(dual_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_interfaces REQUIRED)
find_package(custom_image_msg REQUIRED)
find_package(app_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# include_directories(
#   include
#   ${CMAKE_CURRENT_SOURCE_DIR}/../../install/chat_robot_cpp/include/chat_robot_cpp
# )

# === 新增：OpenSSL 本地支持 ===
# set(OPENSSL_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/x86_64/openssl_3_x_x)
# === 新增：根据架构选择第三方库路径 ===
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(OPENSSL_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/openssl_3_x_x)
  set(WEBP_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/webp)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(OPENSSL_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/x86_64/openssl_3_x_x)
  set(WEBP_LOCAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/x86_64/webp)
endif()


include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # 关键！必须添加生成器路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/app_msgs/include/app_msgs
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/arm_interfaces/include/arm_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/custom_image_msg/include/custom_image_msg/custom_image_msg
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/chat_robot_cpp/include/chat_robot_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/dual_arm_interfaces/include/dual_arm_interfaces/dual_arm_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial_interfaces/include/serial_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  # ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openssl_3_x_x/include
  ${OPENSSL_LOCAL_ROOT}/include
  ${WEBP_LOCAL_ROOT}/include
)

# === 启用 cpp-httplib 的 OpenSSL 支持 ===
add_definitions(-DCPPHTTPLIB_OPENSSL_SUPPORT)

message(STATUS "Include directories: ${CMAKE_CURRENT_SOURCE_DIR}/../../install/chat_robot_cpp/include/chat_robot_cpp")
# # 包含生成的头文件目录
# include_directories(
#   include
#   ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PcmAudio.msg"
  DEPENDENCIES std_msgs
)

add_executable(chat_node src/chat_node.cpp)
ament_target_dependencies(chat_node
    rclcpp
    rclcpp_action
    std_msgs
    iflytek_msg
    arm_interfaces
    dual_arm_interfaces
    unique_identifier_msgs
    sensor_msgs
    serial_interfaces
    custom_image_msg
    ament_index_cpp
    app_msgs
    tf2
    tf2_ros
)


target_include_directories(chat_node PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/chat_robot_cpp/include/chat_robot_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/arm_interfaces/include/arm_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/dual_arm_interfaces/include/dual_arm_interfaces/dual_arm_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial_interfaces/include/serial_interfaces
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/custom_image_msg/include/custom_image_msg/custom_image_msg
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/app_msgs/include/app_msgs
)

# 在 add_executable 之后，添加这行：
target_link_libraries(chat_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # 这是关键！
  ${OPENSSL_LOCAL_ROOT}/lib64/libssl.so
  ${OPENSSL_LOCAL_ROOT}/lib64/libcrypto.so
  ${WEBP_LOCAL_ROOT}/lib/libsharpyuv.so
  ${WEBP_LOCAL_ROOT}/lib/libwebp.so
  ${WEBP_LOCAL_ROOT}/lib/libwebpmux.so
  ${WEBP_LOCAL_ROOT}/lib/libwebpdemux.so
)

set_target_properties(chat_node PROPERTIES
  INSTALL_RPATH "${OPENSSL_LOCAL_ROOT}/lib64:${WEBP_LOCAL_ROOT}/lib"
  BUILD_RPATH "${OPENSSL_LOCAL_ROOT}/lib64:${WEBP_LOCAL_ROOT}/lib"
  LINK_FLAGS "-Wl,-rpath,${OPENSSL_LOCAL_ROOT}/lib64:${WEBP_LOCAL_ROOT}/lib"
)

install(TARGETS chat_node
    DESTINATION lib/${PROJECT_NAME}
)

# 定义数据文件目录
set(DATA_DIR "data")

# 安装 data 目录及其内容
install(
  DIRECTORY ${DATA_DIR}/
  DESTINATION share/${PROJECT_NAME}/data
  FILES_MATCHING PATTERN "*.data"
  PATTERN "*.txt"
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
