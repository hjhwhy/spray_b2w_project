cmake_minimum_required(VERSION 3.8)
project(z1_arm_controller_cpp)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArm.srv"
  "srv/MoveArmWithRPY.srv"
  DEPENDENCIES geometry_msgs std_msgs
)
# --- 设置 z1_sdk 路径 ---
set(Z1_SDK_PATH "/home/test/z1_sdk" CACHE PATH "Path to z1_sdk installation")

# --- 查找头文件 --- 将 z1_sdk 的 include 目录添加到包含路径
include_directories(${Z1_SDK_PATH}/include)

# --- 查找库文件 -- 查找 libz1_sdk.so 或 libz1_sdk.a
find_library(Z1_SDK_LIBRARY
  NAMES Z1_SDK_aarch64
  PATHS ${Z1_SDK_PATH}/lib
  NO_DEFAULT_PATH
)
if(NOT Z1_SDK_LIBRARY)
  message(FATAL_ERROR "无法找到 z1_sdk 库文件 (libz1_sdk.so 或 libz1_sdk.a) 在 ${Z1_SDK_PATH}/lib")
endif()

#
#add_executable(z1_arm_controller_node src/z1_arm_move_node.cpp)
# z1_arm_controller_node.cpp 使用 Service 替代 Subscription
add_executable(z1_arm_controller_node src/z1_arm_controller_node.cpp)

# --- 链接库 ---
ament_target_dependencies(z1_arm_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
)

# 链接 z1_sdk 库
target_link_libraries(z1_arm_controller_node ${Z1_SDK_LIBRARY})
# 链接服务生成的目标
# 获取类型支持目标名称
rosidl_get_typesupport_target(_ts_target z1_arm_controller_cpp "rosidl_typesupport_cpp")
# 手动链接
target_link_libraries(z1_arm_controller_node ${_ts_target})

# --- 安装 ---
install(TARGETS
  z1_arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- ament 包配置 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
