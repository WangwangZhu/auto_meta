# coding:utf-8
'''*****************************************************************************************************
# FileName    : 
# FileFunction: 订阅来自控制算法的控制信号，然后发送到底盘 CAN 总线上。
# Comments    :
*****************************************************************************************************'''
import sys
import rclpy
from canlib import kvadblib
from canlib import Frame
from canlib import canlib
from rclpy.node import Node
from chassis_msg.msg import ADUBodyCmd
from chassis_msg.msg import ADUDriveCmd

class ChassisCommunicationReceive(Node):
    '''*****************************************************************************************************
    - Class Name  : ChassisCommunicationReceive
    - Function    : 
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    *****************************************************************************************************'''
    def __init__(self,
                 channel=0,
                 openFlags=canlib.canOPEN_ACCEPT_VIRTUAL,
                 bitrate=canlib.canBITRATE_500K,
                 bitrateFlags=canlib.canDRIVER_NORMAL,
                 dbc_filename='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/auto_meta/src/chasis_modules/chassis_communication/chassis_communication/nezha.dbc'):
        super().__init__('chassis_communication_receive')
        
        self.maximum_velocity_limitation = 50 # 车辆允许最大速度限制

        # 创建 ROS 2 网络上的控制信号消息
        self.adu_drive_cmd_msg = ADUDriveCmd()
        self.adu_body_cmd_msg = ADUBodyCmd()

        self.database_dbc = kvadblib.Dbc(filename=dbc_filename)
        # 从数据库提取要用的报文 创建和dbc绑定的帧对象
        self.adu_drive_cmd_frame = self.database_dbc.get_message_by_name('ADU_DriveCmd')
        self.adu_body_cmd_frame = self.database_dbc.get_message_by_name('ADU_BodyCmd')
        
        print("adu drive cmd data:")
        print(self.adu_drive_cmd_frame._can_data)

        
        self.channel_number = channel
        self.openFlags = openFlags
        self.bitrate = bitrate
        self.bitrateFlags = bitrateFlags
        
        self.shake_flag_count = 50
        
        self.flash_count = 100
        
        self.drive_count = 500

        """"""""""""""""""""""""
        '''ADU_DriveCmd'''
        """"""""""""""""""""""""
        # 订阅器将 ROS 2 网络上的信号收下来，接受键盘监控程序的使能与失能信号，控制使能与失能
        self.subscription_vehicle_control_mode = self.create_subscription(
            ADUDriveCmd,
            'vehicle_control_mode',
            self.listener_callback_vehicle_control_mode,
            10)
        self.subscription_vehicle_control_mode
        
        self.subscription_vehicle_control_signals_gear = self.create_subscription(
            ADUDriveCmd,
            'vehicle_control_signals_gear',
            self.listener_callback_vehicle_control_signals_gear,
            10
        )
        
        # 订阅器将 ROS 2 网络上的控制信号收下来，接收控制信号
        self.subscription_vehicle_control_signals_gas_brake_steer = self.create_subscription(
            ADUDriveCmd,
            'vehicle_control_signals_gas_brake_steer',
            self.listener_callback_vehicle_control_signals_gas_brake_steer,
            10)
        # prevent unused variable warning
        self.subscription_vehicle_control_signals_gas_brake_steer
        
        """"""""""""""""""""""""
        '''ADU_BodyCmd'''
        """"""""""""""""""""""""
        # TODO:

        # 底盘控制信号循环计数器
        self.rolling_counter_adu_drive_cmd = 0
        self.rolling_counter_adu_body_cmd = 0

        # 定时器将 CAN 数据按照协议频率发送出去
        self.timer_period_adu_drive_control = 0.02  # seconds
        self.timer_adu_drive_control = self.create_timer(self.timer_period_adu_drive_control, self.timer_callback_adu_drive_control)

        self.timer_period_adu_body_control = 0.1
        self.timer_adu_body_control = self.create_timer(self.timer_period_adu_body_control, self.timer_callback_adu_body_control)

        self.ch = self.open_channel()

    '''**************************************************************************************
    - FunctionName: 
    - Function    : 将 ROS 2 网络上的数据取下来放到成员变量里面
    - Inputs      : msg : 订阅器里设置的消息类型
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''

    def listener_callback_vehicle_control_signals_gas_brake_steer(self, msg):
        self.adu_drive_cmd_msg.adu_brk_stoke_req = msg.adu_brk_stoke_req
        self.adu_drive_cmd_msg.adu_gas_stoke_req = msg.adu_gas_stoke_req
        self.adu_drive_cmd_msg.adu_str_whl_ang_req = msg.adu_str_whl_ang_req
        # self.adu_drive_cmd_msg.adu_gear_req = msg.adu_gear_req
        # self.get_logger().info('I heard: "%s"' % msg.steering_mode)
    
    def listener_callback_vehicle_control_signals_gear(self, msg):
        self.adu_drive_cmd_msg.adu_brk_stoke_req = msg.adu_brk_stoke_req
        self.adu_drive_cmd_msg.adu_gear_req = msg.adu_gear_req
        # self.get_logger().info('I heard: "%s"' % msg.steering_mode)

    def listener_callback_vehicle_control_mode(self, msg):
        self.adu_drive_cmd_msg.adu_hozl_dsbl = msg.adu_hozl_dsbl
        self.adu_drive_cmd_msg.adu_lgt_dsbl = msg.adu_lgt_dsbl
        # self.adu_drive_cmd_msg.adu_gear_req = msg.adu_gear_req
        # self.get_logger().info('I heard: "%s"' % msg.steering_mode)

    def timer_callback_adu_drive_control(self):
        '''**************************************************************************************
        - FunctionName: timer_callback_adu_drive_control
        - Function    : 将成员变量里的值打包为 CAN 数据，然后发送到 CAN 网络上去
        - Inputs      : None
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        self.adu_drive_cmd_frame_msg = self.adu_drive_cmd_frame.bind()
        
        self.adu_drive_cmd_frame_msg.ADU_BrkStokeReq.phys = self.adu_drive_cmd_msg.adu_brk_stoke_req
        # self.adu_drive_cmd_frame_msg.ADU_BrkStokeReq.phys = 20
        self.adu_drive_cmd_frame_msg.ADU_GasStokeReq.phys = self.adu_drive_cmd_msg.adu_gas_stoke_req
        self.adu_drive_cmd_frame_msg.ADU_StrWhlAngReq.phys = self.adu_drive_cmd_msg.adu_str_whl_ang_req
        
        self.adu_drive_cmd_frame_msg.ADU_HozlDsbl.phys = self.adu_drive_cmd_msg.adu_hozl_dsbl
        self.adu_drive_cmd_frame_msg.ADU_LgtDsbl.phys = self.adu_drive_cmd_msg.adu_lgt_dsbl
        
        self.adu_drive_cmd_frame_msg.ADU_GearReq.phys = self.adu_drive_cmd_msg.adu_gear_req
        
        self.adu_drive_cmd_frame_msg.ADU_LimGasSpd.phys = self.maximum_velocity_limitation
        
        self.adu_drive_cmd_frame_msg.ADU_DrvCmd_RollCnt.phys = self.rolling_counter_adu_drive_cmd
        
        if self.shake_flag_count <= 0:
            self.adu_drive_cmd_frame_msg.ADU_ShakeReq.phys = 2
        else:
            self.adu_drive_cmd_frame_msg.ADU_ShakeReq.phys = 0
            self.shake_flag_count -= 1
        
        check_sum = self.adu_drive_cmd_frame_msg._data[0] ^ self.adu_drive_cmd_frame_msg._data[1] ^ self.adu_drive_cmd_frame_msg._data[2] ^ self.adu_drive_cmd_frame_msg._data[3] ^ self.adu_drive_cmd_frame_msg._data[4] ^ self.adu_drive_cmd_frame_msg._data[5] ^ self.adu_drive_cmd_frame_msg._data[6]
        self.adu_drive_cmd_frame_msg.ADU_DrvCmd_CheckSum.raw = check_sum
        
        self.rolling_counter_adu_drive_cmd += 1
        if self.rolling_counter_adu_drive_cmd == 16:
            self.rolling_counter_adu_drive_cmd = 0

        self.ch.write(self.adu_drive_cmd_frame_msg._frame)
        
        # print("adu drive cmd data:")
        # for i in range(8):
        #     print(self.adu_drive_cmd_frame_msg._data[i],end="; ")
        # print()
        # print(check_sum)    
        
        self.get_logger().info("Steer: {}, Brake: {}, Acce {}.".format(self.adu_drive_cmd_frame_msg.ADU_StrWhlAngReq.phys, 
                                                                       self.adu_drive_cmd_frame_msg.ADU_BrkStokeReq.phys, 
                                                                       self.adu_drive_cmd_frame_msg.ADU_GasStokeReq.phys))
                                                                                                                    
    def timer_callback_adu_body_control(self):
        '''**************************************************************************************
        - FunctionName: timer_callback_adu_body_control
        - Function    : 将成员变量里的值打包为 CAN 数据，然后发送到 CAN 网络上去
        - Inputs      : None
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        self.adu_body_cmd_frame_msg = self.adu_body_cmd_frame.bind()
        
        self.adu_body_cmd_frame_msg.ADU_Horn.raw = self.adu_body_cmd_msg.adu_horn
        self.adu_body_cmd_frame_msg.ADU_TurnRLamp.phys = self.adu_body_cmd_msg.adu_turn_rlamp
        self.adu_body_cmd_frame_msg.ADU_TurnLLamp.phys = self.adu_body_cmd_msg.adu_turn_llamp
        self.adu_body_cmd_frame_msg.ADU_DblFlashLamp.phys = self.adu_body_cmd_msg.adu_dbl_flash_lamp
        self.adu_body_cmd_frame_msg.ADU_BodyCmd_RollCnt.phys = self.rolling_counter_adu_body_cmd
        
        check_sum = self.adu_body_cmd_frame_msg._data[0] ^ self.adu_body_cmd_frame_msg._data[1] ^ self.adu_body_cmd_frame_msg._data[2] ^ self.adu_body_cmd_frame_msg._data[3] ^ self.adu_body_cmd_frame_msg._data[4] ^ self.adu_body_cmd_frame_msg._data[5] ^ self.adu_body_cmd_frame_msg._data[6]
        self.adu_body_cmd_frame_msg.ADU_BodyCmd_CheckSum.raw = check_sum

        self.ch.write(self.adu_body_cmd_frame_msg._frame)

        self.rolling_counter_adu_body_cmd += 1
        if self.rolling_counter_adu_body_cmd == 16:
            self.rolling_counter_adu_body_cmd = 0

    def open_channel(self):
        '''**************************************************************************************
        - FunctionName: openChannel()
        - Function    : 打开CAN通道，设置CAN通信的波特率并且启动通道
        - Inputs      : None
        - Outputs     : 返回的内容用于供关闭通道的函数调用,并且用于发送函数与接受函数的通道选择
        - Comments    : None
        **************************************************************************************'''
        ch = canlib.openChannel(self.channel_number, self.openFlags)
        num_channels = canlib.getNumberOfChannels()
        print("num_channels:%d" % num_channels)
        print("%d. %s (%s / %s)" % (num_channels, canlib.ChannelData(0).channel_name, canlib.ChannelData(0).card_upc_no, canlib.ChannelData(0).card_serial_no))
        ch.setBusOutputControl(self.bitrateFlags)
        ch.setBusParams(self.bitrate)
        ch.busOn()
        return ch

def main(args=None):
    '''*****************************************************************************************************
    - FunctionName: main
    - Function    : 
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    *****************************************************************************************************'''
    rclpy.init(args=args)
    chassis_communication_receive = ChassisCommunicationReceive()
    rclpy.spin(chassis_communication_receive)
    chassis_communication_receive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
