#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import math
import time
from typing import Optional, Tuple

class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        
        # 初始化云台角度（水平角，仰角）
        self.current_pan = 0.0    # 水平角度（-180°~180°）
        self.current_tilt = -30.0 # 仰角（-30°~90°）
        self.pan_step = 30.0      # 水平旋转步长（°）
        self.tilt_step = 15.0     # 仰角调整步长（°）
        self.scan_complete = False # 是否完成360°扫描
        self.target_lost = False   # 是否丢失目标
        
        # 激光雷达数据
        self.lidar_distance = 0.0  # 摄像头正前方距离（米）
        
        # PID控制参数（水平/仰角）
        self.kp_pan = 0.1
        self.ki_pan = 0.01
        self.kd_pan = 0.05
        
        self.kp_tilt = 0.1
        self.ki_tilt = 0.01
        self.kd_tilt = 0.05
        
        # 误差累积
        self.prev_error_pan = 0.0
        self.integral_pan = 0.0
        
        self.prev_error_tilt = 0.0
        self.integral_tilt = 0.0
        
        # ROS2 发布器（控制云台）
        self.pub_pan_tilt = self.create_publisher(Vector3, '/pan_tilt_control', 10)
        
        # ROS2 订阅器（激光雷达距离）
        self.sub_lidar = self.create_subscription(
            Float32,
            '/lidar_distance',
            self.lidar_callback,
            10
        )
        
        # 定时器（控制循环）
        self.timer = self.create_timer(0.5, self.control_loop)  # 0.5秒周期
    
    def lidar_callback(self, msg: Float32):
        """更新激光雷达距离数据"""
        self.lidar_distance = msg.data
    
    def yolo_detection_callback(self, yolo_results):
        """
        处理YOLOv5检测结果（假设输入是Pandas DataFrame格式）
        返回值: (target_found, target_x, target_y, target_width, target_height)
        """
        if yolo_results.empty:
            return False, 0, 0, 0, 0
        
        # 假设取第一个检测到的目标
        target = yolo_results.iloc[0]
        target_x_center = (target['xmin'] + target['xmax']) / 2
        target_y_center = (target['ymin'] + target['ymax']) / 2
        target_width = target['xmax'] - target['xmin']
        target_height = target['ymax'] - target['ymin']
        
        return True, target_x_center, target_y_center, target_width, target_height
    
    def calculate_pan_tilt_adjustment(
        self,
        img_width: int,
        img_height: int,
        target_x: float,
        target_y: float,
        target_width: float,
        target_height: float,
        lidar_distance: float
    ) -> Tuple[float, float]:
        """
        计算云台调整角度（PID控制）
        返回: (pan_adjustment, tilt_adjustment)
        """
        # 计算目标中心与图像中心的偏差（像素）
        error_pan = target_x - (img_width / 2)
        error_tilt = target_y - (img_height / 2)
        
        # PID控制（水平）
        self.integral_pan += error_pan
        derivative_pan = error_pan - self.prev_error_pan
        pan_adjustment = (
            self.kp_pan * error_pan +
            self.ki_pan * self.integral_pan +
            self.kd_pan * derivative_pan
        )
        self.prev_error_pan = error_pan
        
        # PID控制（仰角）
        self.integral_tilt += error_tilt
        derivative_tilt = error_tilt - self.prev_error_tilt
        tilt_adjustment = (
            self.kp_tilt * error_tilt +
            self.ki_tilt * self.integral_tilt +
            self.kd_tilt * derivative_tilt
        )
        self.prev_error_tilt = error_tilt
        
        # 转换为角度（假设摄像头FOV=60°）
        pan_adjustment_deg = (pan_adjustment / img_width) * 60.0
        tilt_adjustment_deg = (tilt_adjustment / img_height) * 60.0
        
        return pan_adjustment_deg, tilt_adjustment_deg
    
    def move_pan_tilt(self, pan: float, tilt: float):
        """发送云台控制指令（ROS2）"""
        msg = Vector3()
        msg.x = pan    # 水平角度
        msg.y = tilt   # 仰角
        msg.z = 0.0    # 保留字段
        self.pub_pan_tilt.publish(msg)
        self.get_logger().info(f"Moving pan={pan:.1f}°, tilt={tilt:.1f}°")
    
    def control_loop(self):
        """主控制逻辑"""
        # 模拟YOLOv5检测结果（实际使用时替换为真实数据）
        yolo_results = self.get_yolo_results()  # 替换为你的YOLOv5接口
        target_found, target_x, target_y, target_w, target_h = self.yolo_detection_callback(yolo_results)
        
        if not target_found:
            self.target_lost = True
            if not self.scan_complete:
                # 水平旋转30°
                self.current_pan += self.pan_step
                if abs(self.current_pan) >= 180.0:
                    self.scan_complete = True
            else:
                # 仰角提升15°
                self.current_tilt += self.tilt_step
                if self.current_tilt > 90.0:
                    self.current_tilt = -30.0  # 重置仰角
                self.scan_complete = False
                self.current_pan = 0.0  # 重置水平角
            
            self.move_pan_tilt(self.current_pan, self.current_tilt)
        else:
            self.target_lost = False
            self.scan_complete = False
            
            # 计算调整角度
            img_width, img_height = 640, 480  # 假设图像分辨率
            pan_adj, tilt_adj = self.calculate_pan_tilt_adjustment(
                img_width, img_height,
                target_x, target_y,
                target_w, target_h,
                self.lidar_distance
            )
            
            # 更新云台角度
            self.current_pan += pan_adj
            self.current_tilt += tilt_adj
            
            # 限制角度范围
            self.current_pan = max(-180.0, min(180.0, self.current_pan))
            self.current_tilt = max(-30.0, min(90.0, self.current_tilt))
            
            self.move_pan_tilt(self.current_pan, self.current_tilt)

def main(args=None):
    rclpy.init(args=args)
    tracker = TargetTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()