import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from chapt4_interfaces.srv import FaceDetector
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        
        # 检查摄像头是否可用
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            self.get_logger().error('无法打开摄像头！请检查摄像头是否正确连接')
            return
            
        self.get_logger().info('成功打开摄像头')
        
        # 创建定时器，定期从摄像头获取图像
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 创建人脸检测服务的客户端
        self.face_detect_client = self.create_client(FaceDetector, 'face_detect')
        self.get_logger().info('等待人脸检测服务...')
        if not self.face_detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('人脸检测服务在5秒内未启动，请确保face_detect_node正在运行')
            return
        
        self.get_logger().info('摄像头节点已启动，开始处理')
        self.is_processing = False  # 添加标志位避免重复处理

    def timer_callback(self):
        if self.is_processing:
            return
            
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error('无法读取摄像头画面')
            return

        self.get_logger().debug('成功获取摄像头画面')
        
        try:
            # 将OpenCV图像转换为ROS消息
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.get_logger().debug('成功将图像转换为ROS消息')
            
            # 创建服务请求
            request = FaceDetector.Request()
            request.image = img_msg
            
            # 调用人脸检测服务
            self.is_processing = True
            self.get_logger().debug('发送人脸检测请求')
            future = self.face_detect_client.call_async(request)
            future.add_done_callback(self.face_detect_callback)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时发生错误: {str(e)}')
            self.is_processing = False

    def face_detect_callback(self, future):
        try:
            response = future.result()
            self.get_logger().debug('收到人脸检测响应')
            
            # 获取当前摄像头画面
            ret, frame = self.camera.read()
            if not ret:
                self.get_logger().error('在回调中无法读取摄像头画面')
                return
            
            # 在图像上绘制检测到的人脸
            for i in range(response.number):
                top = response.top[i]
                right = response.right[i]
                bottom = response.bottle[i]
                left = response.left[i]
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
            
            # 显示检测结果
            cv2.imshow('Face Detection', frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('用户按下q键，准备退出...')
                rclpy.shutdown()
            
            self.get_logger().info(f'检测到 {response.number} 个人脸，用时: {response.use_time:.2f} 秒')
            
        except Exception as e:
            self.get_logger().error(f'处理人脸检测结果时出错: {str(e)}')
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