from setuptools import find_packages, setup

package_name = 'demo_python_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/resource", ['resource/default.jpg', 'resource/test1.png']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python',
        'numpy'
    ],
    zip_safe=True,
    maintainer='ws',
    maintainer_email='ws@todo.todo',
    description='Python ROS2 demo with camera, face detection, and lidar simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = demo_python_service.camera_node:main',
            'face_detect_node = demo_python_service.face_detect_node:main',
            'lidar_node = demo_python_service.lidar_node:main',
        ],
    },
)
