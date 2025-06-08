import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from cv_bridge import CvBridge
import cv2
import os
import sys
sys.path.append(os.path.expanduser("~/face_recognition_evn/lib/python3.12/site-packages"))
import face_recognition
from ament_index_python.packages import get_package_share_directory
import time


class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.bridge = CvBridge()
        self.number_of_times_to_upsample=1
        self.model="hog"
        self.default_image_path= get_package_share_directory('demo_python_service') + "/resource/test1.png"
        self.get_logger().info("Face detect client node has been started")
        self.image=cv2.imread(self.default_image_path)

    def send_request(self):
        while self.client.wait_for_service(1.0) == False:
            self.get_logger().warn("Waiting for server Face Detect")
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        future=self.client.call_async(request)
        def result_callback(future):
            response=future.result()
            self.get_logger().info("Response: %s" % response.number)
            self.show_response(response)
            # while future.done() == False:
            #     time.sleep(1.1)
        future.add_done_callback(result_callback)

    def show_response(self, response):
        for i in range(response.number):
            cv2.rectangle(self.image, (response.left[i], response.top[i]), 
            (response.right[i], response.bottle[i]), (0, 0, 255), 2)
        cv2.imshow("Face Detection", self.image)
        cv2.waitKey(0)

def main():
    rclpy.init()
    node=FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()