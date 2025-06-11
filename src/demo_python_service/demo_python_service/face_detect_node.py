import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
sys.path.append(os.path.expanduser("~/face_recognition_evn/lib/python3.12/site-packages"))
import face_recognition
from ament_index_python.packages import get_package_share_directory
import time
from chapt4_interfaces.srv import FaceDetector
from chapt4_interfaces.msg import FacePosition


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)
        self.bridge = CvBridge()
        self.face_pub = self.create_publisher(FacePosition, 'face_position', 10)
        self.number_of_times_to_upsample=1
        self.model="hog"
        self.default_image_path= get_package_share_directory('demo_python_service') + "/resource/default.jpg"
        self.get_logger().info("Face detect node has been started")
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    # default_image_path = get_package_share_directory('demo_python_service') + "/resource/default.jpg"
    # print(default_image_path)
    # image=cv2.imread(default_image_path)
    # face_locations = face_recognition.face_locations(image)
    # for top, right, bottom, left in face_locations:
    #     cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
    # cv2.imshow("Face Detection", image)
    # cv2.waitKey(0)
 

    def face_detect_callback(self, request, response):
        start_time = self.get_clock().now().nanoseconds
        try:
            if request.image.data:
                cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            else:
                cv_image = cv2.imread(self.default_image_path)
                self.get_logger().info("No image received, using default image")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            number = len(faces)
            top, right, bottom, left = [], [], [], []
            for (x, y, w, h) in faces:
                left.append(int(x))
                top.append(int(y))
                right.append(int(x + w))
                bottom.append(int(y + h))
            use_time = (self.get_clock().now().nanoseconds - start_time) / 1e9
            response.number = number
            response.top = top
            response.right = right
            response.bottom = bottom
            response.left = left
            response.use_time = float(use_time)
            msg = FacePosition()
            msg.number = number
            msg.top = top
            msg.right = right
            msg.bottom = bottom
            msg.left = left
            msg.use_time = float(use_time)
            self.face_pub.publish(msg)
            self.get_logger().info(f'检测到 {number} 个人脸，用时: {use_time:.2f} 秒')
        except Exception as e:
            self.get_logger().error(f'人脸检测失败: {str(e)}')
            response.number = 0
            response.top = []
            response.right = []
            response.bottom = []
            response.left = []
            response.use_time = 0.0
        return response

def main():
    rclpy.init()
    node = FaceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，准备退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()