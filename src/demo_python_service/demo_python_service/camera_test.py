import cv2

def test_camera():
    # 尝试打开摄像头
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头！")
        return
    
    print("摄像头已成功打开，按 'q' 键退出")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取画面！")
            break
            
        cv2.imshow('Camera Test', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_camera() 