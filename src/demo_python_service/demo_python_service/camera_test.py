import cv2
import time

def test_camera():
    print("正在尝试打开摄像头...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头！")
        return
    
    # 打印摄像头的属性
    print("\n摄像头属性：")
    print(f"帧宽度: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
    print(f"帧高度: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    # 尝试设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("\n摄像头已成功打开，正在尝试读取画面...")
    
    # 尝试多次读取画面
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            print(f"成功读取第 {i+1} 帧")
            print(f"帧大小: {frame.shape}")
            
            # 保存一帧图像用于检查
            test_image_path = "camera_test_frame.jpg"
            cv2.imwrite(test_image_path, frame)
            print(f"已保存测试图像到: {test_image_path}")
            
            # 显示图像
            cv2.imshow('Camera Test', frame)
            print("已尝试显示图像，请检查是否有窗口弹出")
            
            # 等待更长时间以确保窗口能显示
            key = cv2.waitKey(3000)  # 等待3秒
            if key & 0xFF == ord('q'):
                break
        else:
            print(f"读取第 {i+1} 帧失败")
        time.sleep(0.5)  # 每次读取间隔0.5秒
    
    print("\n测试完成，正在释放资源...")
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n请检查：")
    print("1. 是否有窗口弹出？")
    print("2. 是否生成了测试图像文件？")
    print("3. 如果生成了测试图像，请查看图像内容是否正常")

if __name__ == '__main__':
    test_camera() 