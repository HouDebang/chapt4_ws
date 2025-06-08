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
        ('share/' + package_name+"/resource",['resource/default.jpg','resource/test1.png']),
    ],
    # Add face_recognition to required dependencies
    install_requires=['setuptools', 'face_recognition'],  # <-- Modified line
    zip_safe=True,
    maintainer='ws',
    maintainer_email='ws@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'learn_face_recognize = demo_python_service.learn_face_recognize:main',
            'face_detect_node = demo_python_service.face_detect_node:main',
            'face_detect_client_node = demo_python_service.face_detect_client_node:main',
        ],
    },
)
