# coding:utf-8
'''*****************************************************************************************************
# FileName    : keyBoardMonitor.py
# FileFunction: 用于键盘输入监控，由众多的状态机写成
# Comments    :
*****************************************************************************************************'''

from __future__ import print_function
import sys 
import rclpy
from rclpy.node import Node
import pyxhook
from chassis_msg.msg import ADUBodyCmd
from chassis_msg.msg import ADUDriveCmd
from std_msgs.msg import Int8
import time

'''*****************************************************************************************************
- ClassName   : 
- Function    : 广播节点类
- Inputs      : None
- Outputs     : None
- Comments    : None
*****************************************************************************************************'''

class KeyBoardMonitor(Node):
    '''**************************************************************************************
    - FunctionName: __init__
    - Function    : 类初始化
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    def __init__(self):
        super().__init__('key_board_monitor')
        
        # 创建 ROS 2 网络上的控制信号消息
        self.adu_drive_cmd_msg = ADUDriveCmd()
        self.adu_body_cmd_msg = ADUBodyCmd()

        self.running = True
        
        self.publisher_vehicle_control_mode = self.create_publisher(
            ADUDriveCmd,
            'vehicle_control_mode',
            10)
        
        self.hookman = pyxhook.HookManager()
        self.hookman.KeyDown = self.kbenent 
        self.hookman.start()

        timer_period = 0.02  # seconds
        self.timer_check_chassis_can = self.create_timer(timer_period, self.publish_monitor)
    '''**************************************************************************************
    - FunctionName: kbenent
    - Function    : 键盘监控主程序
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''        
    def kbenent(self, event):
        if event.Key == 'q' or event.Key == 'qq':
            self.adu_drive_cmd_msg.adu_hozl_dsbl = 1
            self.adu_drive_cmd_msg.adu_lgt_dsbl = 1
            self.adu_drive_cmd_msg.adu_gear_req = 1
            self.get_logger().info("Autonomous Mode Disabled!!!")
            self.publisher_vehicle_control_mode.publish(self.adu_drive_cmd_msg)
            time.sleep(0.02)
            
        if event.Key == 'e' or event.Key == 'ee':
            self.adu_drive_cmd_msg.adu_hozl_dsbl = 0
            self.adu_drive_cmd_msg.adu_lgt_dsbl = 0
            self.get_logger().info("Autonomous Mode Enabled!!!")
            self.publisher_vehicle_control_mode.publish(self.adu_drive_cmd_msg)
            time.sleep(0.02)
            
        if event.Ascii == 32:
            self.running = False
       
    def publish_monitor(self):
        self.publisher_vehicle_control_mode.publish(self.adu_drive_cmd_msg)

'''*****************************************************************************************************
- FunctionName: main
- Function    : 
- Inputs      : None
- Outputs     : None
- Comments    : None
*****************************************************************************************************'''
def main(args=None):
    rclpy.init(args=args)

    key_board_monitor = KeyBoardMonitor()

    rclpy.spin(key_board_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_board_monitor.destroy_node()
    key_board_monitor.hookman.cancel()
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
