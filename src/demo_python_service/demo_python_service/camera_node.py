import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from chapt4_interfaces.srv import FaceDetector, LidarDistance
from chapt4_interfaces.msg import FacePosition, LidarDistance as LidarDistanceMsg
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            self.get_logger().error('无法打开摄像头！请检查摄像头是否正确连接')
            return
        self.get_logger().info('成功打开摄像头')

        # 服务客户端
        self.face_detect_client = self.create_client(FaceDetector, 'face_detect')
        self.lidar_client = self.create_client(LidarDistance, 'lidar_distance_srv')
        self.get_logger().info('等待人脸检测和激光测距服务...')
        if not self.face_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('人脸检测服务未启动')
            return
        if not self.lidar_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('激光测距服务未启动')
            return
        self.get_logger().info('服务已就绪')

        # 订阅话题
        self.face_position = None
        self.lidar_distance = None
        self.create_subscription(FacePosition, 'face_position', self.face_position_callback, 10)
        self.create_subscription(LidarDistanceMsg, 'lidar_distance', self.lidar_distance_callback, 10)

        # 定时器，定期处理摄像头画面
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.is_processing = False

    def face_position_callback(self, msg):
        self.face_position = msg

    def lidar_distance_callback(self, msg):
        self.lidar_distance = msg.distance

    def timer_callback(self):
        if self.is_processing:
            return
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error('无法读取摄像头画面')
            return
        try:
            # 发送人脸检测服务请求
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            request = FaceDetector.Request()
            request.image = img_msg
            self.is_processing = True
            future = self.face_detect_client.call_async(request)
            future.add_done_callback(lambda fut: self.face_detect_done_callback(fut, frame.copy()))
        except Exception as e:
            self.get_logger().error(f'处理图像时发生错误: {str(e)}')
            self.is_processing = False

    def face_detect_done_callback(self, future, frame):
        try:
            response = future.result()
            # 请求距离服务
            lidar_future = self.lidar_client.call_async(LidarDistance.Request())
            lidar_future.add_done_callback(lambda fut: self.lidar_done_callback(fut, frame, response))
        except Exception as e:
            self.get_logger().error(f'人脸检测服务回调出错: {str(e)}')
            self.is_processing = False

    def lidar_done_callback(self, future, frame, face_response):
        try:
            lidar_response = future.result()
            # 绘制人脸框
            for i in range(face_response.number):
                top = face_response.top[i]
                right = face_response.right[i]
                bottom = face_response.bottom[i]
                left = face_response.left[i]
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            # 绘制距离信息
            distance = lidar_response.distance
            cv2.putText(frame, f'Distance: {distance:.2f} m', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # 显示画面
            cv2.imshow('Camera Face & Lidar', frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('用户按下q键，准备退出...')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'距离服务回调出错: {str(e)}')
        finally:
            self.is_processing = False

    def __del__(self):
        if hasattr(self, 'camera'):
            self.camera.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，准备退出...')
    finally:
        if hasattr(node, 'camera'):
            node.camera.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 