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


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)
        self.bridge = CvBridge()
        self.number_of_times_to_upsample=1
        self.model="hog"
        self.default_image_path= get_package_share_directory('demo_python_service') + "/resource/default.jpg"
        self.get_logger().info("Face detect node has been started")
    # default_image_path = get_package_share_directory('demo_python_service') + "/resource/default.jpg"
    # print(default_image_path)
    # image=cv2.imread(default_image_path)
    # face_locations = face_recognition.face_locations(image)
    # for top, right, bottom, left in face_locations:
    #     cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
    # cv2.imshow("Face Detection", image)
    # cv2.waitKey(0)
 

    def face_detect_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info("No image received, using default image")
        start_time=time.time()
        self.get_logger().info("Start face detection")
        face_locations = face_recognition.face_locations(cv_image,number_of_times_to_upsample=self.number_of_times_to_upsample,model=self.model)
        response.use_time=time.time()-start_time
        response.number=len(face_locations)
        for top, right, bottle, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottle.append(bottle)
            response.left.append(left)
        return response
def main():
    rclpy.init()
    node=FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()