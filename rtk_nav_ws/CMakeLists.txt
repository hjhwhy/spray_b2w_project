cmake_minimum_required(VERSION 3.8)
project(ins_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)
pkg_check_modules(PROJ REQUIRED proj)

include_directories(${PROJ_INCLUDE_DIRS})
link_directories(${PROJ_LIBRARY_DIRS})

add_executable(ins_parser src/ins_parser.cpp)
ament_target_dependencies(ins_parser rclcpp std_msgs geometry_msgs sensor_msgs   tf2
  tf2_geometry_msgs)

add_executable(dog_controller src/dog_controller.cpp)
ament_target_dependencies(dog_controller rclcpp geometry_msgs sensor_msgs std_msgs tf2
  tf2_geometry_msgs)

target_link_libraries(ins_parser ${GeographicLib_LIBRARIES} ${PROJ_LIBRARIES})

install(TARGETS
  ins_parser
  dog_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
