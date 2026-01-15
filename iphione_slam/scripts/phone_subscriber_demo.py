#!/usr/bin/env python3
import cv2
import zmq
import numpy as np
import time
from phone import PhoneSubscriber
import signal
import sys
from PIL import Image
import io

def main():
    ip = "10.192.1.51"
    port = 8000
    
    # 创建订阅者实例
    subscriber = None
    try:
        subscriber = PhoneSubscriber(ip, port)
        
        
        # 持续接收消息
        while True:
            try:
                time_stamp, color_img, depth_img, depth_width, depth_height, local_pose, global_pose, velocity, rotation_rate = subscriber.subscribeMessage(timeout=100)

                
                
                # 显示RGB图像
                if len(color_img) > 0:
                    try:
                        # 将bytes转换为PIL图像
                        color_pil = Image.open(io.BytesIO(color_img))
                        color_array = np.array(color_pil)
                        
                        # 转换颜色格式从RGB到BGR（OpenCV格式）
                        if len(color_array.shape) == 3 and color_array.shape[2] == 3:
                            color_bgr = cv2.cvtColor(color_array, cv2.COLOR_RGB2BGR)
                        else:
                            color_bgr = color_array
                        
                        # 使用OpenCV显示RGB图像
                        cv2.imshow('Color Image', color_bgr)
                        
                    except Exception as e:
                        print(f"处理彩色图像时出错: {e}")
                
                # 显示深度图像
                if len(depth_img) > 0 and depth_width > 0 and depth_height > 0:
                    try:
                        # 将bytes转换为numpy数组 - 深度图像是uint16格式
                        depth_array = np.frombuffer(depth_img, dtype=np.uint16)
                        depth_numpy = depth_array.reshape(depth_height, depth_width)
                     
                        
                        # 将uint16深度数据归一化到0-255用于可视化
                        if depth_numpy.max() > depth_numpy.min():
                            # 线性归一化
                            depth_normalized = ((depth_numpy - depth_numpy.min()) / (depth_numpy.max() - depth_numpy.min()) * 255).astype(np.uint8)
                        else:
                            depth_normalized = np.zeros(depth_numpy.shape, dtype=np.uint8)
                        
                        # 应用伪彩色映射（COLORMAP_JET: 蓝色=近，红色=远）
                        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                        
                        # 显示深度图像
                        cv2.imshow('Depth Image', depth_colored)
                        
                    except Exception as e:
                        print(f"处理深度图像时出错: {e}")
                
                # 非阻塞等待键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC键退出
                    break
                
            except zmq.ZMQError as e:
                # print(f"ZMQ错误: {e}")
                time.sleep(0.01)
            except Exception as e:
                print(f"接收消息时发生意外错误: {type(e).__name__}: {e}")
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
      
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 确保在退出时关闭订阅者
        if subscriber:
            print("关闭订阅者连接...")
            subscriber.close()
            print("连接已关闭")
        
        # 关闭OpenCV窗口
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 
