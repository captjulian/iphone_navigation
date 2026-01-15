#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class TFToOdometry(object):
    def __init__(self):
        rospy.init_node("tf_to_odometry")

        # parameters
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.robot_frame = rospy.get_param("~robot_frame", "robot")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.rate_hz = rospy.get_param("~rate", 30.0)

        # publisher
        self.odom_pub = rospy.Publisher(
            self.odom_topic, Odometry, queue_size=10
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("TF to Odometry node started.")

    def run(self):
        while not rospy.is_shutdown():
            try:
                # lookup transform: world -> robot
                trans = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.robot_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1)
                )

                odom_msg = self.transform_to_odometry(trans)
                self.odom_pub.publish(odom_msg)

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(
                    2.0, "Waiting for TF %s -> %s",
                    self.world_frame, self.robot_frame
                )

            self.rate.sleep()

    def transform_to_odometry(self, trans):
        odom = Odometry()

        odom.header.stamp = trans.header.stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.robot_frame

        # position
        odom.pose.pose.position.x = trans.transform.translation.x
        odom.pose.pose.position.y = trans.transform.translation.y
        odom.pose.pose.position.z = trans.transform.translation.z

        # orientation
        odom.pose.pose.orientation = trans.transform.rotation

        return odom


if __name__ == "__main__":
    node = TFToOdometry()
    node.run()
