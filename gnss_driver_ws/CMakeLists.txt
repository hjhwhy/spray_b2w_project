cmake_minimum_required(VERSION 3.8)
project(gnss_driver)

# 设置 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(tf2_ros REQUIRED)
#find_package(PROJ REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

# 查找 GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)

include_directories(${PROJ_INCLUDE_DIRS})
link_directories(${PROJ_LIBRARY_DIRS})

add_executable(pub_rtk_save_pt_node src/pub_rtk_save_pt_node.cpp)

add_executable(convert_coords src/test.cpp)
target_include_directories(convert_coords PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(convert_coords PRIVATE ${PROJ_LIBRARIES})

# 链接库
target_link_libraries(pub_rtk_save_pt_node
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PROJ_LIBRARIES}
)

# 链接 ROS2 包
ament_target_dependencies(pub_rtk_save_pt_node
  rclcpp
  sensor_msgs
  nav_msgs  
  geometry_msgs
  tf2
  nmea_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS pub_rtk_save_pt_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)
# 安装 config 文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()

