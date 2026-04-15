cmake_minimum_required(VERSION 3.8)
project(rs485_node)

# 查找包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# 添加可执行文件
add_executable(rs485_node src/rs485_node.cpp)
target_link_libraries(rs485_node ${CMAKE_THREAD_LIBS_INIT})

# 链接库
ament_target_dependencies(rs485_node
  rclcpp
  std_msgs
  std_srvs  # 必须包含
)

install(TARGETS
  rs485_node
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

# 导出
ament_package()
