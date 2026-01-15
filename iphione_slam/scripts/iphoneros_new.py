#!/usr/bin/env python3
from asmagic import ARDataSubscriber
from asmagic import IMUDataSubscriber
import rospy
import numpy as np
import cv2
import io
from PIL import Image

from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo


class PhoneROSBridge:
    def __init__(self):
        rospy.init_node("phone_ros_bridge")

        ip = rospy.get_param("~ip", "10.192.1.51")

        self.subar = ARDataSubscriber(ip)
        self.subimu = IMUDataSubscriber(ip)
        self.bridge = CvBridge()

        # ROS publishers
        self.rgb_pub   = rospy.Publisher("/camera/color/image_raw", ROSImage, queue_size=1)
        self.depth_pub = rospy.Publisher("/camera/depth/image_raw", ROSImage, queue_size=1)
        self.imu_pub   = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.pose_pub  = rospy.Publisher("/camera/pose", PoseStamped, queue_size=10)
        self.pub_camera = rospy.Publisher("/camera/depth/camera_info", CameraInfo, queue_size=10)

        rospy.loginfo("Phone ROS Bridge started")

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                
                data = self.subar.get()
                data_imu = self.subimu.get()

                stamp = rospy.Time.from_sec(data.timestamp)
                #print(len(global_pose))
                #if data:
                #print(f"Accelerometer (G): {data.accelerometer}")
                #        print(f"Gyroscope (rad/s): {data.gyroscope}")
                #rospy.loginfo(f"Received message: timestamp={timestamp}")
                #rospy.loginfo(f"Color image size: {len(color_img_bytes)}, Depth image size: {len(depth_img_bytes)}")
                #rospy.loginfo(f"Velocity: {velocity}, Rotation rate: {rotation_rate}")
                #rospy.loginfo(f"Global pose length: {len(data.global_pose) if data.global_pose is not None else 'None'}")
                self.publish_pose(data.global_pose, stamp)
                self.publish_imu(data_imu.accelerometer, data_imu.gyroscope, stamp)
                self.publish_depth(data.depth_array , data.depth_width, data.depth_height, stamp)
                self.publish_rgb(data.color_array, stamp)
                self.publish_info(data.camera_intrinsics, stamp)
                

            except Exception:
                pass

            rate.sleep()
    
    def publish_info(self, camera_intrinsics, stamp):
        
        scale_x = 256/1920
        scale_y = 192/1440
        
        fx = camera_intrinsics[0]*scale_x
        fy = camera_intrinsics[4]*scale_y
        cx = camera_intrinsics[6]*scale_x
        cy = camera_intrinsics[7]*scale_y
        
        msg = CameraInfo()

        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"

        # 图像分辨率（必须填）
        msg.width = 256
        msg.height = 192

        # 相机内参矩阵 K
        msg.K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # 若无畸变，设为 plumb_bob + 全 0
        msg.distortion_model = "plumb_bob"
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

        # R（rectification matrix）
        msg.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # P（projection matrix）
        msg.P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.pub_camera.publish(msg)


    def publish_rgb(self, img_array, stamp):
        #if len(img_bytes) == 0:
        #    return
        #print("image_get")
        #pil_img = Image.open(io.BytesIO(img_bytes))
        #rgb = np.array(pil_img)
        
        msg = self.bridge.cv2_to_imgmsg(img_array, encoding="rgb8")
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_frame"
        self.rgb_pub.publish(msg)
        
    #def publish_rgb(self, img_bytes, stamp):
    #    if len(img_bytes) == 0:
    #        rospy.logwarn("RGB image empty, skipping")
    #        return
    #    try:
    #        pil_img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    #        rgb = np.array(pil_img, dtype=np.uint8)
    #        print("RGB image shape:", rgb.shape, "dtype:", rgb.dtype)

    #        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
    #        msg.header.stamp = stamp
    #        msg.header.frame_id = "camera_color_frame"
    #        self.rgb_pub.publish(msg)
    #        print("RGB published")

    #    except Exception as e:
    #        rospy.logwarn(f"publish_rgb error: {e}")

    def publish_depth(self, depth_array, w, h, stamp):
        if w == 0 or h == 0:
            return

        #depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(h, w)
        depth_meters = depth_array.astype(np.float32) / 10000.0
        #msg = self.bridge.cv2_to_imgmsg(depth_array, encoding="16UC1")
        msg = self.bridge.cv2_to_imgmsg(depth_meters, encoding="32FC1")
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        self.depth_pub.publish(msg)

    def publish_imu(self, velocity, rotation_rate, stamp):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"

        if len(rotation_rate) >= 3:
            imu.angular_velocity.x = rotation_rate[0]
            imu.angular_velocity.y = rotation_rate[1]
            imu.angular_velocity.z = rotation_rate[2]

        if len(velocity) >= 3:
            imu.linear_acceleration.x = velocity[0]
            imu.linear_acceleration.y = velocity[1]
            imu.linear_acceleration.z = velocity[2]

        self.imu_pub.publish(imu)

    def publish_pose(self, pose, stamp):
        if len(pose) < 7:
            return
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"

        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]

        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]

        self.pose_pub.publish(msg)

    def shutdown(self):
        self.subscriber.close()


if __name__ == "__main__":
    try:
        node = PhoneROSBridge()
        node.spin()
    except rospy.ROSInterruptException:
        pass
