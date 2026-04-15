#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_point_publisher')
        
        # 1. 创建发布器
        self.pub = self.create_publisher(PointCloud2, '/unacquired_points', 10)
        
        # 2. 定义测试点 (作为成员变量)
        # 这里模拟几个获取到的点 (x, y, z)
        self.test_points = [
            [1.0, 2.0, 0.0],
            [1.5, 2.5, 0.1],
            [2.0, 2.0, 0.0],
            [2.5, 2.5, 0.2],
            [3.0, 3.0, 0.0]
        ]
        
        # 3. 创建定时器
        timer_period = 1.0  # 秒
        # 正确用法：只传 period 和 callback
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Test Publisher started. Publishing points...')

    def timer_callback(self):
        # 4. 在回调中使用成员变量 self.test_points
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  # 根据你的实际坐标系修改，如 'odom', 'base_link'
        
        # 创建 PointCloud2 消息
        # create_cloud_xyz32 会自动处理 fields 和 data 的布局
        cloud_msg = create_cloud_xyz32(header, self.test_points)
        
        # 发布
        self.pub.publish(cloud_msg)
        self.get_logger().info(f'Published {len(self.test_points)} points to /acquired_points')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
