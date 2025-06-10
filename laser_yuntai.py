import serial
import json
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['font.sans-serif'] = ['SimHei']  # 根据系统安装的字体调整
#云台控制部分
class BaseController:
    def __init__(self, port='/dev/base_controller',baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[Gimbal] 已连接到 {port}")
    def send_command(self,command):
        command_str =json.dumps(command) + "\n"
        self.ser.write(command_str.encode('utf-8'))
        print("[Gimbal] 已发送指令:",command)
    def close(self):
       self.ser.close()
    def move_gimbal(self,angle_x,angle_y=0,speed=50,acc=10):
        cmd= {"T": 133, "X": angle_x, "Y": angle_y, "SPD":speed, "ACC": acc}
        self.send_command(cmd)

#激光雷达实现
def get_lidar_measurement(ser):
    raw_data=b""
    while True:
        if ser.in_waiting>0:
            raw_data+=ser.read(ser.in_waiting)
        start_index=raw_data.find(b'\xAA')
        if start_index!=-1 and len(raw_data)-start_index>=195:
            packet=raw_data[start_index:start_index+195]
            raw_data=raw_data[start_index+195:]
            measurements=[]
            data_list=list(packet)
        for i in range(10,195,15):
            if i+1<len(data_list):
                high=data_list[i+1]
                low=data_list[i]
                distance=(high<<8)|low
                measurements.append(distance)
        if measurements:
            average_distance=sum(measurements)/len(measurements)
            return average_distance
        time.sleep(0.05)

#主流程
if __name__=='__main__':
    gimbal=BaseController('/dev/base_controller',115200)
    try:
        lidar_ser=serial.Serial('/dev/wheeltec_controller',230400,timeout=1)
        print("[Lidar]已连接到/dev/wheeltec_controller")
    except Exception as e:
        print("[Lidar]已连接失败",e)
        exit(1)
    # 扫描参数设置(单位:度)
    horiz_start=-30 #水平起始角
    horiz_end = 30# 水平结束角
    horiz_step =10# 水平步进
    vert_start=-10# 垂直起始角
    vert_end=40# 垂直结束角
    vert_step =10# 垂直步进
    wait_time = 0.5 # 每次云台运动后等待时间(秒)
    # 存储扫描数据，数据格式:(horiz_angle,vert_angle,distance)
    scan_data = []
    total_steps = ((horiz_end - horiz_start) // horiz_step + 1) * ((vert_end -vert_start) // vert_step + 1)
    current_step = 1
    print("开始3D扫描..")
    for vert_angle in range(vert_start, vert_end + 1, vert_step):
        gimbal.move_gimbal(angle_x = horiz_start, angle_y = vert_angle)
        time.sleep(5)
        for horiz_angle in range(horiz_start, horiz_end + 1, horiz_step):
            print(f"\n[Scan] Step {current_step}/{total_steps}，移动到 水平 {horiz_angle}°，垂直 {vert_angle}°")
            gimbal.move_gimbal(angle_x=horiz_angle, angle_y = vert_angle)
            time.sleep(1)  # 等待云台到位并稳定
            distance = get_lidar_measurement(lidar_ser)
            print(f"[Scan] 角度 (H={horiz_angle}°,V={vert_angle}°)测得平均距离:{distance:.2f} mm")
            scan_data.append((horiz_angle, vert_angle,distance))
            current_step+=1
            time.sleep(0.5)
    gimbal.close()
    lidar_ser.close()
    if scan_data:
        xs, ys, zs = [], [], []
        for h_angle, v_angle, d in scan_data:
            theta = math.radians(h_angle)  # 水平角(方位角)
            phi = math.radians(v_angle)# 垂直角(仰角)
            phi = math.radians(v_angle)
            # 球坐标转换为直角坐标
            x = d* math.cos(phi) * math.cos(theta)
            y = d* math.cos(phi) * math.sin(theta)
            z = d* math.sin(phi)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            # 绘制3D点云
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='扫描点')
        # 绘制传感器自身位置
        ax.scatter(0, 0, 0, c='red', marker='^', s=100, label='传感器位置')
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title("3D 激光雷达点云")
        ax.legend()
        plt.show()
    else:
        print("未采集到任何数据点")

