cmake_minimum_required(VERSION 3.8)
project(spray_path_planner)

# ROS2 CMake 设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 查找包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 如果你有自定义 .srv 文件，必须添加这个
find_package(rosidl_default_generators REQUIRED)

# 声明你的 .srv 接口文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetNextWaypoint.srv"
  "srv/SetStartPoint.srv"  # 新增
  DEPENDENCIES geometry_msgs  
)

# 添加可执行文件
add_executable(spray_path_planner_node src/spray_path_planner_node.cpp)

ament_target_dependencies(spray_path_planner_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  sensor_msgs
  )

# 链接库
target_link_libraries(spray_path_planner_node
  rclcpp::rclcpp
)
# 链接服务生成的目标
# 获取类型支持目标名称
rosidl_get_typesupport_target(_ts_target spray_path_planner "rosidl_typesupport_cpp")
# 手动链接
target_link_libraries(spray_path_planner_node ${_ts_target})

# 安装可执行文件
install(TARGETS
  spray_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件（可选）
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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