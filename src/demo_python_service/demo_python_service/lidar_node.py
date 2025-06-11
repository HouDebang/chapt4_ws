import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from chapt4_interfaces.srv import LidarDistance
from chapt4_interfaces.msg import LidarDistance as LidarDistanceMsg
import random

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        # 创建服务
        self.srv = self.create_service(LidarDistance, 'lidar_distance_srv', self.lidar_distance_callback)
        # 创建发布者
        self.lidar_pub = self.create_publisher(LidarDistanceMsg, 'lidar_distance', 10)
        # 定时器，定时发布距离
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('激光测距仪节点已启动')
        self.current_distance = 0.0

    def get_lidar_distance(self):
        # 这里用随机数模拟距离，实际应用中请替换为真实读取代码
        return round(random.uniform(0.5, 5.0), 2)

    def timer_callback(self):
        self.current_distance = self.get_lidar_distance()
        msg = LidarDistanceMsg()
        msg.distance = float(self.current_distance)
        self.lidar_pub.publish(msg)
        self.get_logger().debug(f'发布距离: {self.current_distance} m')

    def lidar_distance_callback(self, request, response):
        response.distance = float(self.current_distance)
        self.get_logger().info(f'服务响应距离: {self.current_distance} m')
        return response

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，准备退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 