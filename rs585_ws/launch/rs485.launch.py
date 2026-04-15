from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('rs485_node')

    # 加载参数文件
    params_file = os.path.join(pkg_dir, 'config', 'rs485_params.yaml')

    return LaunchDescription([
        Node(
            package='rs485_node',
            executable='rs485_node',  # 替换为你的可执行文件名
            name='rs485_node',
            output='screen',
            parameters=[params_file],
            # 可选：重启策略
            respawn=True,
        )
    ])