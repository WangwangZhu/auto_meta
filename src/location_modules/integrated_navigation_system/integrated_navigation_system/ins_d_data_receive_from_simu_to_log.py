#coding=utf-8
'''*****************************************************************************************************
# FileName    : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''
import os
import sys
import csv
import math
import time
import rclpy
import serial
import struct
import datetime
import logging
import collections  
import numpy as np 

from time import sleep
from rclpy.node import Node
from rclpy.qos import QoSProfile
from collections import OrderedDict
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped  
from tf2_ros.transform_broadcaster import TransformBroadcaster
from nav_msgs.msg._odometry import Odometry

# from . import ins_data_parse_base_class

class ins_d_data_receive_from_simu_to_log(Node):
    '''**************************************************************************************
    - FunctionName: Main
    - Function    : main
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    def __init__(self,
                 data_storage_folder='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/' + 'auto_meta/run_log/',
                #  data_storage_folder='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2]  + '/auto_meta/comfort_dataset/ins_d_data/', 
                 data_map_sub_name = "_ins_data_log.csv",
                 node_name="ins_d_data_receive_from_simu_to_log"):
        super().__init__(node_name)
        self._seq_ros2 = 0
        self._odom_msgs = Odometry()
        self._qos = QoSProfile(depth=10)
        # self._odom_msgs_publisher = self.create_publisher(Odometry, "ins_d_of_vehicle_pose", self._qos)
        self._odom_msgs_subscription = self.create_subscription(Odometry, "ins_d_of_vehicle_pose", self.ind_receive_callback, self._qos)
        
        self.date_now = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        
        # self._tf_publisher = TransformBroadcaster(self)
        if not os.path.exists(data_storage_folder):
                    os.makedirs(data_storage_folder)
        self.data_map_name = data_storage_folder + 'ins_d_data_' + time.strftime("%Y_%m_%d",time.localtime(time.time()))
        
        if not os.path.exists(self.data_map_name):
            os.makedirs(self.data_map_name)
        
        self.data_map_name = self.data_map_name + '/' + self.date_now + data_map_sub_name

        # self.is_open_flag = self.serial_.isOpen()
        
        self.ins_d_feedback_csv_logger = logging.getLogger("longitudinal feedback logger")
        self.ins_d_feedback_csv_logger.setLevel(logging.DEBUG)
        self.ins_d_feedback_file_handler = logging.FileHandler(self.data_map_name)
        self.ins_d_feedback_file_handler.setLevel(logging.DEBUG)
        self.ins_d_feedback_frmt = logging.Formatter('%(asctime)s,%(message)s')
        self.ins_d_feedback_file_handler.setFormatter(self.ins_d_feedback_frmt)
        self.ins_d_feedback_csv_logger.addHandler(self.ins_d_feedback_file_handler)

    def ind_receive_callback(self, msg):
        '''**************************************************************************************
        - FunctionName: 
        - Function    : 
        - Inputs      : 
        - Outputs     : None
        - Comments    : INS OPVT2AHR 使用的协议类型
                        规定：传递消息使用的 xyz 满足x沿着车辆纵向方向，向前为正，从车屁股看，y指向左侧，
                        ins——d本体的坐标系：y沿着车辆纵向方向，向前为正，从车屁股看，x指向右侧
        **************************************************************************************'''
        # C x, D y, 
        # E heading_radians, F pitch_radians, G roll_radians
        # 三个角速度 H I J
        # 三个线速度 K L M 
        # 纵向加速度 N 
        self.ins_d_feedback_csv_logger.info("{},{},{},{},{},{},{},{},{},{},{},{}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, 
                                                           msg.pose.covariance[1], msg.pose.covariance[2], msg.pose.covariance[3],
                                                           msg.twist.twist.angular.x * 57.29578, msg.twist.twist.angular.y * 57.29578, msg.twist.twist.angular.z * 57.29578,
                                                           msg.twist.twist.linear.x * 3.6, msg.twist.twist.linear.y * 3.6, msg.twist.twist.linear.z * 3.6,
                                                           msg.pose.covariance[0]))
        # r = Rotation.from_quat
        
        # with open(self.data_map_name, 'w', newline='') as f:
        #     self.writer = csv.DictWriter(f, self.headers)
        #     self.writer.writeheader()   
        #     while rclpy.ok():
        #         self.receive_from_buffer() 
        #         data = self.block_parse()
        #         self.writer.writerow(data) 
                                
        #         self._odom_msgs.header.frame_id = "odom"
        #         self._odom_msgs.header.stamp = self.time_now
                
        #         self._odom_msgs.pose.pose.position.x = self.local_x
        #         self._odom_msgs.pose.pose.position.y = self.local_y
        #         self._odom_msgs.pose.pose.position.z = self.altitude

        #         r = Rotation.from_euler('zyx',[self.heading_radians, self.roll_radians, self.pitch_radians])
        #         self._orientation = r.as_quat()

        #         self._odom_msgs.pose.pose.orientation.x = self._orientation[0]
        #         self._odom_msgs.pose.pose.orientation.y = self._orientation[1]
        #         self._odom_msgs.pose.pose.orientation.z = self._orientation[2]
        #         self._odom_msgs.pose.pose.orientation.w = self._orientation[3]
        #         # 车辆坐标系
        #         self._odom_msgs.twist.twist.linear.y = self.vehicle_lat_speed #  * 3.6
        #         self._odom_msgs.twist.twist.linear.x = self.vehicle_lon_speed # * 3.6
                
        #         self._odom_msgs.twist.twist.linear.z = self.vertical_speed # * 3.6
                
        #         self._odom_msgs.twist.twist.angular.x = self.gyroX / 57.29578
        #         self._odom_msgs.twist.twist.angular.y = self.gyroY / 57.29578
        #         self._odom_msgs.twist.twist.angular.z = self.gyroZ / 57.29578
                
        #         acceleration_ = self.accY * math.cos(self.pitch_radians) - self.accZ * math.sin(self.pitch_radians)
                
        #         # 本身该类型消息，没有加速度信号，为了传递方便，占用方差信号的空间，传递加速度，为了方便，同时， 传递一下车辆三个状态角度。
        #         self._odom_msgs.pose.covariance[0] = float(acceleration_)
        #         self._odom_msgs.pose.covariance[1] = self.heading_radians
        #         self._odom_msgs.pose.covariance[2] = self.pitch_radians
        #         self._odom_msgs.pose.covariance[3] = self.roll_radians
        #         self._odom_msgs.pose.covariance[4] = float(self.accX)
        #         self._odom_msgs.pose.covariance[5] = self.gps_time
                
        #         self._odom_msgs_publisher.publish(self._odom_msgs)
        #         self.get_logger().info("ins data send out =>  heading degree: " + str(self._odom_msgs.pose.covariance[1]*57))

'''*****************************************************************************************************
- FunctionName: main
- Function    : 
- Inputs      : None
- Outputs     : None
- Comments    : None
*****************************************************************************************************'''

def main(args=None):
    rclpy.init(args=args)
    
    ins_d_data_receive_from_simu_to_log_ = ins_d_data_receive_from_simu_to_log()
    # ins_d_data_receive_from_simu_to_log_.data_parse() # 这里直接进入while死循环了，所以后面的启动回调并没有开始
    
    rclpy.spin(ins_d_data_receive_from_simu_to_log_)
    
    ins_d_data_receive_from_simu_to_log_.destroy_node()
    rclpy.shutdown()

'''*****************************************************************************************************
- FunctionName: main
- Function    : 
- Inputs      : None
- Outputs     : None
- Comments    : None
*****************************************************************************************************'''
if __name__ == '__main__':
    main()