#!/usr/bin/env python3

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

from phone import PhoneSubscriber

class PhoneROSBridge:
    def __init__(self):
        rospy.init_node("phone_ros_bridge")

        ip = rospy.get_param("~ip", "10.192.1.51")
        port = rospy.get_param("~port", 8000)

        self.subscriber = PhoneSubscriber(ip, port)
        self.bridge = CvBridge()

        # ROS publishers
        self.rgb_pub   = rospy.Publisher("/camera/color/image_raw", ROSImage, queue_size=1)
        self.depth_pub = rospy.Publisher("/camera/depth/image_raw", ROSImage, queue_size=1)
        self.imu_pub   = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.pose_pub  = rospy.Publisher("/camera/pose", PoseStamped, queue_size=10)

        rospy.loginfo("Phone ROS Bridge started")

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                (
                    timestamp,
                    color_img_bytes,
                    depth_img_bytes,
                    depth_width,
                    depth_height,
                    local_pose,
                    global_pose,
                    velocity,
                    rotation_rate
                ) = self.subscriber.subscribeMessage(timeout=50)

                stamp = rospy.Time.from_sec(timestamp)
                #print(len(global_pose))

                #rospy.loginfo(f"Received message: timestamp={timestamp}")
                #rospy.loginfo(f"Color image size: {len(color_img_bytes)}, Depth image size: {len(depth_img_bytes)}")
                #rospy.loginfo(f"Velocity: {velocity}, Rotation rate: {rotation_rate}")
                #rospy.loginfo(f"Global pose length: {len(global_pose) if global_pose is not None else 'None'}")
                self.publish_pose(global_pose, stamp)
                self.publish_imu(velocity, rotation_rate, stamp)
                self.publish_depth(depth_img_bytes, depth_width, depth_height, stamp)
                self.publish_rgb(color_img_bytes, stamp)
                

            except Exception:
                pass

            rate.sleep()

    def publish_rgb(self, img_bytes, stamp):
        if len(img_bytes) == 0:
            return
        #print("image_get")
        pil_img = Image.open(io.BytesIO(img_bytes))
        rgb = np.array(pil_img)
        
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
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

    def publish_depth(self, depth_bytes, w, h, stamp):
        if len(depth_bytes) == 0 or w == 0 or h == 0:
            return

        depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(h, w)

        msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
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
