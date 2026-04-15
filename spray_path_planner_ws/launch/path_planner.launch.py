from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('spray_path_planner')

    # 加载参数文件
    params_file = os.path.join(pkg_dir, 'config', 'path_planner.yaml')

    return LaunchDescription([
        Node(
            package='spray_path_planner',
            executable='spray_path_planner_node',  # 替换为你的可执行文件名
            name='spray_path_planner',
            output='screen',
            parameters=[params_file],
            # 可选：重启策略
            # respawn=True,
        )
    ])