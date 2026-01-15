#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import tf

class FirstPoseAsWorldWithTF:
    def __init__(self):
        rospy.init_node('iphone_slam_tf')

        self.first_T_inv = None

        self.sub = rospy.Subscriber(
            '/camera/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=10
        )

        self.pose_pub = rospy.Publisher(
            '/camera/pose_world',
            PoseStamped,
            queue_size=10
        )

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_0 = tf2_ros.TransformBroadcaster()

        R_ow = np.array([
            [0, 0, -1],  # Z -> X'
            [-1, 0, 0],  # X -> Y'
            [0, 1, 0]   # Y -> Z'
            ])
        self.T_ow = np.eye(4)
        self.T_ow[:3, :3] = R_ow

        
        Ry = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
            ])
        
        Rx = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
            ])
        
        myR = Ry @ Rx

        self.T_ro = np.eye(4)
        self.T_ro[:3, :3] = myR
        

        rospy.loginfo("First PoseStamped defines world frame (identity). Publishing Pose + TF.")

    def pose_to_T(self, pose):
        q = pose.orientation
        t = pose.position

        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]
        return T

    def T_to_pose(self, T, msg):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "world"

        pose.pose.position.x = T[0, 3]
        pose.pose.position.y = T[1, 3]
        pose.pose.position.z = T[2, 3]

        q = tft.quaternion_from_matrix(T)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose
    
    def publish_robot_tf(self, T, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "robot"

        tf_msg.transform.translation.x = 0
        tf_msg.transform.translation.y = 0
        tf_msg.transform.translation.z = 0

        quat = tft.quaternion_from_matrix(T)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster_0.sendTransform(tf_msg)

    def publish_tf(self, T, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "odom"

        tf_msg.transform.translation.x = T[0, 3]
        tf_msg.transform.translation.y = T[1, 3]
        tf_msg.transform.translation.z = T[2, 3]

        q = tft.quaternion_from_matrix(T)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)
    
    def T_to_odom(self,T):
        """
        4x4 numpy 矩阵转换为 geometry_msgs/Pose
        """
        p = Point(*T[:3,3])
        q = tf.transformations.quaternion_from_matrix(T)
        q_msg = Quaternion(*q)
        return Pose(position=p, orientation=q_msg)

    def pose_callback(self, msg: PoseStamped):
        T = self.pose_to_T(msg.pose)

        #world
        if self.first_T_inv is None:
            self.first_T_inv = np.linalg.inv(T)
            rospy.loginfo("First frame received → set as world (identity)")
            return

        #world
        T_world = self.first_T_inv @ T
        T_world_new = self.T_ow @ T_world

        #PoseStamped
        #pose_world = self.T_to_pose(T_world_new, msg)
        #self.pose_pub.publish(pose_world)

        #odom
        #pose_odom = self.T_to_odom(T_world_new)
        #odom_msg = Odometry()
        #odom_msg.header.stamp = msg.header.stamp
        #odom_msg.header.frame_id = "world"
        #odom_msg.child_frame_id = "odom"
        #odom_msg.pose.pose = pose_odom

        #self.odom_pub.publish(odom_msg)

        #TF world → odom
        self.publish_tf(T_world_new, msg.header.stamp)
        self.publish_robot_tf(self.T_ro, msg.header.stamp)

if __name__ == '__main__':
    try:
        FirstPoseAsWorldWithTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

