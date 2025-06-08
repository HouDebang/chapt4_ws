import sys
import os
# Add virtual environment's site-packages to Python path
sys.path.append(os.path.expanduser("~/face_recognition_evn/lib/python3.12/site-packages"))

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory

def main():
    default_image_path = get_package_share_directory('demo_python_service') + "/resource/default.jpg"
    print(default_image_path)
    image=cv2.imread(default_image_path)
    face_locations = face_recognition.face_locations(image)
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
    cv2.imshow("Face Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
