from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('gnss_driver')

    # 加载参数文件
    params_file = os.path.join(pkg_dir, 'config', 'gnss_params.yaml')

    return LaunchDescription([
        Node(
            package='gnss_driver',
            executable='pub_rtk_save_pt_node',  # 替换为你的可执行文件名
            name='gnss_driver',
            output='screen',
            parameters=[params_file],
            # 可选：重启策略
            # respawn=True,
        )
    ])