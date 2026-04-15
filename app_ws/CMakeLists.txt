cmake_minimum_required(VERSION 3.5)
project(robot_tcp)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robot_tcp_node src/app_node.cpp)

ament_target_dependencies(robot_tcp_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  std_srvs
)

target_link_libraries(robot_tcp_node
  ${Boost_LIBRARIES}
)

# Install
install(TARGETS robot_tcp_node DESTINATION lib/${PROJECT_NAME})

ament_package()

