#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Imu
import tf

class FirstPoseAsWorldWithTF:
    def __init__(self):
        rospy.init_node('iphone_slam_tf_1')
        
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=10)  # 添加IMU订阅

        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        
        self.odom_pub = rospy.Publisher('/odom_0', Odometry, queue_size=10)

        #self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_1 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_2 = tf2_ros.TransformBroadcaster()

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
        
        R_c_r = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
            ])
        
        R_c_r_z = np.array([
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
            ])
        
        myR  = Ry @ Rx
        self.T_ro = np.eye(4)
        self.T_ro[:3, :3] = myR
        
        myRc = myR @ R_c_r
        myRc = myRc @ R_c_r_z
        self.T_rc = np.eye(4)
        self.T_rc[:3, :3] = myRc
        
        self.gravity_transform = np.identity(4)  # 初始化重力变换矩阵
        self.imu_data = []  # 初始化IMU数据列表
        self.imu_start_time = None  # 初始化IMU数据开始时间
        self.calibration_done = False
        self.gravity_estimation_time = 3.0
        
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
    
    def publish_imu_tf(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "imu_link"

        tf_msg.transform.translation.x = 0
        tf_msg.transform.translation.y = 0
        tf_msg.transform.translation.z = 0

        tf_msg.transform.rotation.x = 0
        tf_msg.transform.rotation.y = 0
        tf_msg.transform.rotation.z = 0
        tf_msg.transform.rotation.w = 1

        self.tf_broadcaster_2.sendTransform(tf_msg)
    
    def publish_robot_tf(self, T, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = T[0, 3]
        tf_msg.transform.translation.y = T[1, 3]
        tf_msg.transform.translation.z = T[2, 3]

        quat = tft.quaternion_from_matrix(T)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster_0.sendTransform(tf_msg)
        
    def publish_camera_tf(self, T, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "camera_depth_frame"

        tf_msg.transform.translation.x = 0
        tf_msg.transform.translation.y = 0
        tf_msg.transform.translation.z = 0

        quat = tft.quaternion_from_matrix(T)
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster_1.sendTransform(tf_msg)
    
    def T_to_odom(self,T):
        """
        4x4 numpy 矩阵转换为 geometry_msgs/Pose
        """
        p = Point(*T[:3,3])
        q = tf.transformations.quaternion_from_matrix(T)
        q_msg = Quaternion(*q)
        return Pose(position=p, orientation=q_msg)

    def odom_callback(self, msg):
        T = self.pose_to_T(msg.pose.pose)

        T_bw = self.gravity_transform @ T
        
        pose_odom = self.T_to_odom(T_bw)
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose_odom

        self.odom_pub.publish(odom_msg)

        #TF world → odom
        if self.gravity_transform is not None and not np.array_equal(self.gravity_transform, np.identity(4)):
            self.publish_robot_tf(T_bw, msg.header.stamp)
            self.publish_camera_tf(self.T_rc, msg.header.stamp)
            self.publish_imu_tf(msg.header.stamp)
        
    def imu_callback(self, msg):
        if self.imu_start_time is None and not self.calibration_done:
            self.imu_start_time = rospy.Time.now()  # 记录IMU数据开始时间

        self.imu_data.append(msg.linear_acceleration)  # 累积IMU数据

        if not self.calibration_done:
            elapsed_time = rospy.Time.now() - self.imu_start_time
            if elapsed_time.to_sec() >= self.gravity_estimation_time:
                self.process_imu_data() 
                self.imu_data = []  
                self.imu_start_time = None
                self.calibration_done = True 

    def process_imu_data(self):
        # 计算平均的重力方向
        #avg_acceleration = np.mean([np.array([acc.x, acc.y, acc.z]) for acc in self.imu_data], axis=0)
        avg_acceleration = np.mean([np.array([-acc.z, acc.y, acc.x]) for acc in self.imu_data], axis=0)
        print(avg_acceleration)
        self.gravity_transform = self.estimate_gravity_transform(avg_acceleration)

    def estimate_gravity_transform(self, acceleration):
        # 估计重力方向并生成变换矩阵
        gravity = acceleration
        gravity /= np.linalg.norm(gravity)  # 归一化重力向量

        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(gravity, z_axis)
        rotation_angle = np.arccos(np.dot(gravity, z_axis))
        
        print(rotation_angle)

        transform = np.identity(4)
        transform[:3, :3] = self.rotation_matrix(rotation_axis, rotation_angle)
        return transform
    
    def rotation_matrix(self, axis, theta):
        # 计算绕轴旋转的旋转矩阵
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                            [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                            [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

if __name__ == '__main__':
    try:
        FirstPoseAsWorldWithTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

