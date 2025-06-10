import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from chapt4_interfaces.srv import FaceDetector
import numpy as np
import serial  # 激光雷达串口库
import time

# 内联实现原laser_yuntai.py的云台控制功能
class BaseController:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        if not self.ser.is_open:
            raise Exception(f"无法打开串口 {port}")

    def move_gimbal(self, angle_x, angle_y, speed, acc):
        # 示例协议：假设云台接收格式为"AX{:.1f}AY{:.1f}S{:d}A{:d}\n"的指令
        cmd = f"AX{angle_x:.1f}AY{angle_y:.1f}S{speed}A{acc}\n"
        self.ser.write(cmd.encode('utf-8'))

    def close(self):
        if self.ser.is_open:
            self.ser.close()

# 内联实现原laser_yuntai.py的雷达测距功能
def get_lidar_measurement(ser):
    # 示例协议：假设雷达返回格式为"DIST:1234.5mm\n"的字符串
    data = ser.readline().decode('utf-8').strip()
    if data.startswith("DIST:"):
        try:
            return float(data.split(":")[1].replace("mm", ""))
        except:
            return 0.0
    return 0.0

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
        
        # 硬件初始化
        # 新增：初始化云台和激光雷达（需根据实际硬件调整端口）
        self.gimbal = BaseController('COM3', 115200)  # Windows串口格式为COMx
        try:
            self.lidar_ser = serial.Serial('COM4', 230400, timeout=1)
            self.get_logger().info("[Lidar] 已连接到COM4")
        except Exception as e:
            self.get_logger().error("[Lidar] 连接失败", e)
            exit(1)
        
        # 新增：图像参数（根据实际摄像头调整）
        self.image_width = 640  # 需与摄像头实际分辨率一致
        self.image_height = 480
        self.angle_per_pixel = 0.1  # 像素到角度的转换系数（需校准）

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
                bottom = response.bottom[i]
                left = response.left[i]
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

            # 新增：处理目标追踪逻辑
            if response.number > 0:
                # 选择第一个检测到的目标（可扩展为选择最大/最近目标）
                idx = 0
                left = response.left[idx]
                top = response.top[idx]
                right = response.right[idx]
                bottom = response.bottom[idx]

                # 计算目标中心坐标
                target_center_x = (left + right) // 2
                target_center_y = (top + bottom) // 2

                # 计算与图像中心的偏移（像素）
                image_center_x = self.image_width // 2
                image_center_y = self.image_height // 2
                dx = target_center_x - image_center_x
                dy = target_center_y - image_center_y

                # 转换为云台调整角度（简单比例控制）
                angle_x = dx * self.angle_per_pixel
                angle_y = dy * self.angle_per_pixel

                # 发送云台调整指令（限制角度范围防止过冲）
                self.gimbal.move_gimbal(
                    angle_x=max(-30, min(30, angle_x)),  # 假设水平范围±30°
                    angle_y=max(-10, min(40, angle_y)),  # 垂直范围-10°~40°
                    speed=30,  # 调整速度
                    acc=5      # 加速度
                )

                # 获取激光雷达测距（需确保云台已稳定）
                time.sleep(0.2)  # 等待云台稳定
                distance = get_lidar_measurement(self.lidar_ser)

                # 在目标框附近显示距离（新增）
                cv2.putText(
                    frame,
                    f"距离: {distance:.2f}mm",
                    (left, top - 10),  # 显示在框上方
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),  # 绿色文字
                    2
                )

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
        self.gimbal.close()  # 新增：关闭云台连接
        self.lidar_ser.close()  # 新增：关闭雷达串口
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