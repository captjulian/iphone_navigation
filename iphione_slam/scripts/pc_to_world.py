#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

from sensor_msgs.msg import PointCloud2


class PointCloudToWorldNode:
    def __init__(self):
        rospy.init_node("pointcloud_to_world")

        # ===== Parameters =====
        self.target_frame = rospy.get_param("~target_frame", "world")
        self.input_topic = rospy.get_param("~input_topic", "/camera/depth/points")
        self.output_topic = rospy.get_param("~output_topic", "/points_world")

        # ===== TF Buffer =====
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ===== ROS Pub/Sub =====
        self.sub = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1
        )

        self.pub = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1
        )

        rospy.loginfo("PointCloudToWorldNode initialized")
        rospy.loginfo(f"Input topic:  {self.input_topic}")
        rospy.loginfo(f"Output topic: {self.output_topic}")
        rospy.loginfo(f"Target frame: {self.target_frame}")

    def cloud_callback(self, cloud_msg: PointCloud2):
        try:
            # 查找 camera_frame -> world 的 TF
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,                 # target
                cloud_msg.header.frame_id,         # source
                rospy.Time(0),                     # latest available
                rospy.Duration(0.1)
            )

            # 执行点云坐标变换
            cloud_world = tf2_sm.do_transform_cloud(
                cloud_msg,
                transform
            )

            # 设置 frame_id
            cloud_world.header.frame_id = self.target_frame

            # 发布
            self.pub.publish(cloud_world)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF transform failed: {e}")


if __name__ == "__main__":
    try:
        node = PointCloudToWorldNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
