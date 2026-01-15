#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import tf
from sensor_msgs.msg import PointCloud2
import threading
from utils import publish_point_cloud, rotation_matrix, dtype_from_fields

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
        
        self.sub = rospy.Subscriber(
            '/camera/depth/points',
            PointCloud2,
            self.pc_callback,
            queue_size=10
        )

        self.pose_pub = rospy.Publisher(
            '/camera/pose_world',
            PoseStamped,
            queue_size=10
        )
        
        self.registered_body_pub = rospy.Publisher('/points_world', PointCloud2, queue_size=10)  # 添加点云发布器

        #self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_1 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster_2 = tf2_ros.TransformBroadcaster()
        
        self.point_list = list()
        self.point_list_lock = threading.Lock()  # 添加锁来保护self.point_list

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
        tf_msg.header.frame_id = "robot"
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
        tf_msg.child_frame_id = "robot"

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
        tf_msg.header.frame_id = "robot"
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
    
    def pc_callback(self, msg):
        timestamp = msg.header.stamp
        points_structured = np.ndarray(shape=(msg.width * msg.height,),
                                       dtype=dtype_from_fields(msg.fields, point_step=msg.point_step),
                                       buffer=msg.data)
        points = np.column_stack([points_structured["x"], points_structured["y"], points_structured["z"]])
        with self.point_list_lock:
            self.point_list.append((points, timestamp))
        

    def pose_callback(self, msg: PoseStamped):
        T = self.pose_to_T(msg.pose)

        #world
        #if self.first_T_inv is None:
        #    self.first_T_inv = np.linalg.inv(T)
        #    rospy.loginfo("First frame received → set as world (identity)")
        #    return

        #world
        #T_world = self.first_T_inv @ T
        #T_world_new = self.T_ow @ T_world
        T_world_new = self.T_ow @ T
        T_world_robot = T_world_new @ self.T_ro
        T_camera = T_world_robot @ self.T_rc
        
        if self.point_list:
            with self.point_list_lock:
                points, timestamp = self.point_list.pop(0)
            N = points.shape[0]
            points_h = np.hstack([points, np.ones((N, 1))])   # (N,4)
            points_transformed_h = (T_camera @ points_h.T).T         # (N,4)
            points_transformed = points_transformed_h[:, :3]
            publish_point_cloud(points_transformed, self.registered_body_pub, 'world', timestamp)

        #TF world → odom
        #self.publish_tf(T_world_new, msg.header.stamp)
        self.publish_robot_tf(T_world_robot, msg.header.stamp)
        #self.publish_robot_tf(self.T_ro, msg.header.stamp)
        self.publish_camera_tf(self.T_rc, msg.header.stamp)
        self.publish_imu_tf(msg.header.stamp)

if __name__ == '__main__':
    try:
        FirstPoseAsWorldWithTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

