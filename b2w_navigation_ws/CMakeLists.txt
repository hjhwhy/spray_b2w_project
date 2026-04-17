cmake_minimum_required(VERSION 3.5)
project(b2w_navigation_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(z1_arm_controller_cpp REQUIRED)
find_package(spray_path_planner REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含路径
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/local/include/ddscxx)
include_directories(${EIGEN3_INCLUDE_DIR})
# --- 设置 Unitree SDK 路径
# set(UNITREE_SDK_ROOT "/home/oneko/projects/spray_b2w_robot_project_greek/unitree_sdk2/install")
#set(UNITREE_IDL_ROOT "/home/hjh/unitree_sdk2/include/unitree")
set(UNITREE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2/install")
# 添加头文件路径（include 在 SDK 根目录）
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${UNITREE_SDK_ROOT}/include
 # ${UNITREE_IDL_ROOT}
)
# 添加库文件搜索路径
link_directories(
  ${UNITREE_SDK_ROOT}/lib
)
# 添加可执行文件
add_executable(b2w_nav_node src/main.cpp)
add_executable(b2w_teleop_node src/b2w_teleop.cpp)
# 导出依赖
ament_target_dependencies(b2w_nav_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  std_srvs
  z1_arm_controller_cpp
  spray_path_planner
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(b2w_teleop_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
)

# 链接库
target_link_libraries(b2w_nav_node
  ${CMAKE_THREAD_LIBS_INIT}
  unitree_sdk2    # 静态库 libunitree_sdk2.a
  ddsc 
  ddscxx
  Eigen3::Eigen
)
target_link_libraries(b2w_teleop_node
  ${CMAKE_THREAD_LIBS_INIT}
  unitree_sdk2
  ddsc 
  ddscxx
)

install(TARGETS b2w_nav_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS b2w_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)


# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)


ament_package()