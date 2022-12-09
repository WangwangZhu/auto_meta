# coding:utf-8
'''*****************************************************************************************************
# FileName    : 
# FileFunction: 订阅来自控制算法的控制信号，然后发送到底盘 CAN 总线上。
# Comments    :
*****************************************************************************************************'''
import os
import sys
import time
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from canlib import canlib
from canlib import Frame
from canlib import kvadblib

from chassis_msg.msg import WVCUBodyStatus
from chassis_msg.msg import WVCUFltCod
from chassis_msg.msg import WVCUHorizontalStatus
from chassis_msg.msg import WVCULongitudinalStatus
from chassis_msg.msg import WVCUVehSphLim

from chassis_msg.msg import ADUBodyCmd
from chassis_msg.msg import ADUDriveCmd

class SafetyMechanism(Node):
    def __init__(self):
        super().__init__("safety_mechanism")
        self.publisher_ = self.create_publisher(ADUDriveCmd,"vehicle_control_mode",10)
        self.subscription_ = self.create_subscription(WVCUHorizontalStatus, "wvcu_horizontal_status", self.emergency_callback,10)
        self.subscription_
        self.eps_steering_control_msg = ADUDriveCmd()

    def emergency_callback(self, msg):
        if msg.wvcu_str_whl_tq > 5.0:
            self.get_logger().info("Manual Mode")
            self.eps_steering_control_msg.adu_hozl_dsbl = 1
            self.eps_steering_control_msg.adu_lgt_dsbl = 1
            # self.eps_steering_control_msg.adu_gear_req = 1
            self.publisher_.publish(self.eps_steering_control_msg)

def main(args = None):
    rclpy.init(args=args)
    safety_mechanism_publisher = SafetyMechanism()
    rclpy.spin(safety_mechanism_publisher)
    safety_mechanism_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
