# coding:utf-8
'''*****************************************************************************************************
# FileName    : 
# FileFunction: 订阅来自控制算法的档位控制信号，产生档位切换逻辑指令，然后发送到底盘 CAN 总线上。
# Comments    :
*****************************************************************************************************'''
# import sys
import rclpy
# from canlib import kvadblib
# from canlib import Frame
# from canlib import canlib
from rclpy.node import Node
# from chassis_msg.msg import ADUBodyCmd
from chassis_msg.msg import ADUDriveCmd
from chassis_msg.msg import ADUGearRequest
from chassis_msg.msg import WVCULongitudinalStatus

class GearController(Node):
    '''*****************************************************************************************************
    - Class Name  : GearController
    - Function    : 
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    *****************************************************************************************************'''
    def __init__(self):
        super().__init__('gear_controller')
        
        # self.maximum_velocity_limitation = 30 # 车辆允许最大速度限制
        
        self.gear_request_acting_flag = 0

        # 订阅控制器的档位请求
        self.adu_gear_request = ADUGearRequest
        self.gear_request_subscription = self.create_subscription(ADUGearRequest, "vehicle_control_signals_gear_request", self.listener_callback_vehicle_gear_request)
        self.gear_request_subscription
        
        # 订阅车辆当前的速度、档位状态
        self.wvcu_longitudinal_status_msg = WVCULongitudinalStatus()
        self.wvcu_longitudinal_status_subscription = self.create_subscription(WVCULongitudinalStatus, "wvcu_longitudinal_status", self.listener_callback_vehicle_longitudinal_status)
        self.wvcu_longitudinal_status_subscription
        
        # 广播档位的控制逻辑
        self.adu_drive_cmd_msg = ADUDriveCmd()
        self.adu_drive_cmd_gear_publisher = self.create_publisher(ADUDriveCmd, 'vehicle_control_signals_gear', 10)
        
    '''**************************************************************************************
    - FunctionName: 
    - Function    : 4个档位的控制逻辑
    - Inputs      : 
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    def gear_request_P(self):
        if self.wvcu_longitudinal_status_msg.wvcu_gear_stat == 0: # 当前已经是 P 档位了，不需要再发控制指令
            return
        else:
            self.gear_request_acting_flag = 1
            self.adu_drive_cmd_msg.adu_brk_stoke_req = 35
            self.adu_drive_cmd_msg.adu_gear_req = 0
            self.adu_drive_cmd_gear_publisher.publish(self.adu_drive_cmd_msg)
            while self.wvcu_longitudinal_status_msg.wvcu_gear_stat != 0:
                self.get_logger().warning("Change to P!!!")
            self.gear_request_acting_flag = 0
            
        
    def gear_request_N(self):
        if self.wvcu_longitudinal_status_msg.wvcu_gear_stat == 1: # 当前已经是 N 档位了，不需要再发控制指令
            return
        else:
            self.gear_request_acting_flag = 1
            self.adu_drive_cmd_msg.adu_brk_stoke_req = 35
            self.adu_drive_cmd_msg.adu_gear_req = 1
            self.adu_drive_cmd_gear_publisher.publish(self.adu_drive_cmd_msg)
            while self.wvcu_longitudinal_status_msg.wvcu_gear_stat != 0:
                self.get_logger().warning("Change to P!!!")
            self.gear_request_acting_flag = 0
        
    def gear_request_R(self):
        if self.wvcu_longitudinal_status_msg.wvcu_gear_stat == 2: # 当前已经是 R 档位了，不需要再发控制指令
            return
        else:
            self.gear_request_acting_flag = 1
            self.adu_drive_cmd_msg.adu_brk_stoke_req = 35
            self.adu_drive_cmd_msg.adu_gear_req = 2
            self.adu_drive_cmd_gear_publisher.publish(self.adu_drive_cmd_msg)
            while self.wvcu_longitudinal_status_msg.wvcu_gear_stat != 0:
                self.get_logger().warning("Change to P!!!")
            self.gear_request_acting_flag = 0
        
    def gear_request_D(self):
        if self.wvcu_longitudinal_status_msg.wvcu_gear_stat == 3: # 当前已经是 D 档位了，不需要再发控制指令
            return
        else:
            self.gear_request_acting_flag = 1
            self.adu_drive_cmd_msg.adu_brk_stoke_req = 35
            self.adu_drive_cmd_msg.adu_gear_req = 3
            self.adu_drive_cmd_gear_publisher.publish(self.adu_drive_cmd_msg)
            while self.wvcu_longitudinal_status_msg.wvcu_gear_stat != 0:
                self.get_logger().warning("Change to P!!!")
            self.gear_request_acting_flag = 0

    
    def listener_callback_vehicle_gear_request(self, msg):
        ''' **************************************************************************************
        - FunctionName: 
        - Function    : 将 ROS 2 网络上的数据取下来放到成员变量里面
        - Inputs      : msg : 订阅器里设置的消息类型
        - Outputs     : None
        - Comments    : None
        ************************************************************************************** '''
        self.adu_gear_request.gear_request = msg.gear_request
        
        # 档位请求与当前的实际档位状态相同，不需要向底盘发送切换档位的命令
        if self.adu_gear_request.gear_request == self.wvcu_longitudinal_status_msg.wvcu_gear_stat:
            return 
        else:
            if self.wvcu_longitudinal_status_msg.wvcu_veh_spd != 0:
                self.get_logger().error("Current velocity is not equal to 0km/h, no response to gear_request!!!")
            else:
                if self.gear_request_acting_flag == 0: 
                    if self.adu_gear_request.gear_request == 0:
                        self.gear_request_P()
                        self.get_logger().info('Gear Request: P')
                    elif self.adu_gear_request.gear_request == 1:
                        self.gear_request_N()
                        self.get_logger().info('Gear Request: N')
                    elif self.adu_gear_request.gear_request == 2:
                        self.gear_request_R()
                        self.get_logger().info('Gear Request: R')
                    elif self.adu_gear_request.gear_request == 3:
                        self.gear_request_D()
                        self.get_logger().info('Gear Request: D')
                    else:
                        self.get_logger().warning("Wrong Gear Request!!!")
            
    '''**************************************************************************************
    - FunctionName: 
    - Function    : 将 ROS 2 网络上的数据取下来放到成员变量里面
    - Inputs      : msg : 订阅器里设置的消息类型
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    def listener_callback_vehicle_longitudinal_status(self, msg):
        self.wvcu_longitudinal_status_msg = msg
        # if self.wvcu_longitudinal_status_msg.wvcu_veh_spd != 0:
        #     self.get_logger().warning("Current velocity is not equal to 0km/h, no response to gear_request!!!")

def main(args=None):
    '''*****************************************************************************************************
    - FunctionName: main
    - Function    : 
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    *****************************************************************************************************'''
    rclpy.init(args=args)
    gear_controller = GearController()
    rclpy.spin(gear_controller)
    gear_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
