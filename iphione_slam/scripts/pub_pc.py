#!/usr/bin/env python3
import os
import sys
import rospy
import threading
import numpy as np
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_matrix
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped

from utils import publish_point_cloud, rotation_matrix, dtype_from_fields

class MeasureList:
    def __init__(self):
        self.point_measures = list()
        self.imu_measures = list()
    
    def append(self, points, timestamp, offsets):
        pass


class PC_Node(object):
    def __init__(self):
        rospy.init_node('point_cloud_0_pub', anonymous=True)
        
        self.loop = rospy.Rate(20)
        self.local_frame = rospy.get_param("~local_frame", 'robot')
        self.global_frame = rospy.get_param("~global_frame", 'world')
        self.map_frame = rospy.get_param("~map_frame", 'world')
        self.num_threads = rospy.get_param("~num_threads", 16)
        self.point_topic = rospy.get_param("~point_topic", '/camera/depth/points')
        self.imu_topic = rospy.get_param("~imu_topic", '/imu/data')
        self.gravity_estimation_time = 3.0
        
        self.pose = np.identity(4)
        
        
        rospy.Subscriber(self.point_topic, PointCloud2, self.scan_callback, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=10)  # 添加IMU订阅
        self.registered_body_pub = rospy.Publisher('/registered_body', PointCloud2, queue_size=10)  # 添加点云发布器
        self.gravity_transform = np.identity(4)  # 初始化重力变换矩阵
        self.imu_data = []  # 初始化IMU数据列表
        self.imu_start_time = None  # 初始化IMU数据开始时间
        self.calibration_done = False
        
        self.point_list = list()
        self.point_list_lock = threading.Lock()  # 添加锁来保护self.point_list

    def scan_callback(self, msg):
        timestamp = msg.header.stamp
        points_structured = np.ndarray(shape=(msg.width * msg.height,),
                                       dtype=dtype_from_fields(msg.fields, point_step=msg.point_step),
                                       buffer=msg.data)
        points = np.column_stack([points_structured["x"], points_structured["y"], points_structured["z"]])

        #print(points_structured["timestamp"])

        # Read timestamp offset from intensity field
        #if "timestamp" in points_structured.dtype.fields:
        #    print("coming here")
        #    offsets = points_structured["timestamp"] - timestamp.to_nsec()

        # 将点云数据存入self.point_list
        with self.point_list_lock:
            self.point_list.append((points, timestamp))
            #self.point_list.append((points, timestamp, offsets))

    def registration_thread(self):
        while not rospy.is_shutdown():
            if self.point_list:
                with self.point_list_lock:
                    points, timestamp = self.point_list.pop(0)

                if self.gravity_transform is not None and not np.array_equal(self.gravity_transform, np.identity(4)):
                    publish_point_cloud(points, self.registered_body_pub, self.local_frame, timestamp)
            else:
                rospy.sleep(0.05)  # 等待新的点云数据

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
        avg_acceleration = np.mean([np.array([acc.z, acc.y, acc.x]) for acc in self.imu_data], axis=0)
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
        transform[:3, :3] = rotation_matrix(rotation_axis, rotation_angle)
        return transform
    
    def integrate_imu(self, msg):
        # TODO: use imu for calibration later
        # 使用IMU数据进行姿态积分
        dt = (msg.header.stamp - self.last_imu_time).to_sec()
        self.last_imu_time = msg.header.stamp

        # 使用简单的欧拉积分法来更新姿态
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        delta_rotation = rotation_matrix(angular_velocity, dt)
        self.imu_pose = self.imu_pose @ delta_rotation

        linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        delta_translation = self.imu_pose[:3, :3] @ linear_acceleration * dt
        self.imu_pose[:3, 3] += delta_translation

    def run(self):
        self.reg_thread = threading.Thread(target=self.registration_thread)
        self.reg_thread.start()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("BIM Evaluation Node is terminated")
        finally:
            self.reg_thread.join()
    
    def publish_transform(self, pose_matrix, timestamp=None):
        # use time stamp from the point cloud
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now() if timestamp is None else timestamp
        transform.header.frame_id = self.global_frame
        transform.child_frame_id = self.local_frame
        transform.transform.translation.x = pose_matrix[0, 3]
        transform.transform.translation.y = pose_matrix[1, 3]
        transform.transform.translation.z = pose_matrix[2, 3]
        q = quaternion_from_matrix(pose_matrix)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)
    
    def rotation_matrix(axis, theta):
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


if __name__ == "__main__":
    myPC = PC_Node()
    myPC.run()