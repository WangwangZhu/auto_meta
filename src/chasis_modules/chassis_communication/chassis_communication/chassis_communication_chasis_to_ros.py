# coding:utf-8
'''*****************************************************************************************************
# FileName    :
# FileFunction: 从底盘CAN网络上读取车辆状态，然后转发到ROS2网络上
# Comments    :
*****************************************************************************************************'''
import os
import sys
import rclpy
import logging
from datetime import datetime
from rclpy.node import Node
from canlib import canlib
from canlib import Frame
from canlib import kvadblib

from chassis_msg.msg import WVCUBodyStatus
from chassis_msg.msg import WVCUFltCod
from chassis_msg.msg import WVCUHorizontalStatus
from chassis_msg.msg import WVCULongitudinalStatus
from chassis_msg.msg import WVCUVehSphLim

class ChassisCommunicationSend(Node):
    '''*************************************************************************************
    - FunctionName:
    - Function    : 广播节点类
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    def __init__(self, 
                 channel=0, 
                 openFlags=canlib.canOPEN_ACCEPT_VIRTUAL, 
                 bitrate=canlib.canBITRATE_500K,
                 bitrateFlags=canlib.canDRIVER_NORMAL, 
                 dbc_filename='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/auto_meta/src/chassis_communication/chassis_communication/JY_OGF_NETA_ADU_CCM_V1.01.dbc'):
        super().__init__('chassis_communication_send')

        timer_period = 1/2000  # seconds
        self.timer_check_chassis_can = self.create_timer(timer_period, self.receive_from_chassis)

        # 创建广播器，将底盘上的反馈信号广播到 ROS 2 网络上
        self.publisher_wvcu_longitudinal_status_feedback = self.create_publisher(WVCULongitudinalStatus, 'wvcu_longitudinal_status', 10)
        self.publisher_wvcu_horizontal_status_feedback = self.create_publisher(WVCUHorizontalStatus, 'wvcu_horizontal_status', 10)
        self.publisher_wvcu_body_status_feedback = self.create_publisher(WVCUBodyStatus, 'wvcu_body_status', 10)
        self.publisher_wvcu_flt_cod_feedback = self.create_publisher(WVCUFltCod, 'wvcu_flt_cod', 10)
        self.publisher_wvcu_veh_sph_lim_feedback = self.create_publisher(WVCUVehSphLim, 'wvcu_veh_sph_lim', 10)

        # 创建 ROS 2 消息对象
        self.wvcu_longitudinal_status_msg = WVCULongitudinalStatus()
        self.wvcu_horizontal_status_msg = WVCUHorizontalStatus()
        self.wvcu_body_status_msg = WVCUBodyStatus()
        self.wvcu_flt_cod_msg = WVCUFltCod()
        self.wvcu_veh_sph_lim_msg = WVCUVehSphLim()

        self.channel_number = channel
        self.openFlags = openFlags
        self.bitrate = bitrate
        self.bitrateFlags = bitrateFlags

        self.database_dbc = kvadblib.Dbc(filename=dbc_filename)
        # 从数据库提取要用的报文 创建和dbc绑定的帧对象
        self.wvcu_longitudinal_status_frame = self.database_dbc.get_message_by_name('WVCU_LongitudinalStatus')
        self.wvcu_horizontal_status_frame = self.database_dbc.get_message_by_name('WVCU_HorizontalStatus')
        self.wvcu_body_status_frame = self.database_dbc.get_message_by_name('WVCU_BodyStatus')
        self.wvcu_flt_cod_frame = self.database_dbc.get_message_by_name('VCU_Speed_Feedback')
        self.wvcu_veh_sph_lim_frame = self.database_dbc.get_message_by_name('VCU_Speed_Feedback')

        self.ch = self.open_channel_kavser()

        self.date_now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        csv_logger_folder = '/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/auto_meta/run_log/'
        if not os.path.exists(csv_logger_folder):
            os.makedirs(csv_logger_folder)

        self.eps_feedback_csv_logger = logging.getLogger("eps feedback logger")
        self.eps_feedback_csv_logger.setLevel(logging.DEBUG)
        self.eps_feedback_file_handler = logging.FileHandler(csv_logger_folder + "eps_feedback_log_" + self.date_now + ".csv")
        self.eps_feedback_file_handler.setLevel(logging.DEBUG)
        self.eps_feedback_frmt = logging.Formatter('%(asctime)s,%(message)s')
        self.eps_feedback_file_handler.setFormatter(self.eps_feedback_frmt)
        self.eps_feedback_csv_logger.addHandler(self.eps_feedback_file_handler)
        
        self.longitudinal_feedback_csv_logger = logging.getLogger("longitudinal feedback logger")
        self.longitudinal_feedback_csv_logger.setLevel(logging.DEBUG)
        self.longitudinal_feedback_file_handler = logging.FileHandler(csv_logger_folder + "longitudinal_feedback_log_" + self.date_now + ".csv")
        self.longitudinal_feedback_file_handler.setLevel(logging.DEBUG)
        self.longitudinal_feedback_frmt = logging.Formatter('%(asctime)s,%(message)s')
        self.longitudinal_feedback_file_handler.setFormatter(self.longitudinal_feedback_frmt)
        self.longitudinal_feedback_csv_logger.addHandler(self.longitudinal_feedback_file_handler)

    def open_channel_kavser(self):
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
        print("%d. %s (%s / %s)" % (num_channels, canlib.ChannelData(0).channel_name,
                                    canlib.ChannelData(0).card_upc_no, canlib.ChannelData(0).card_serial_no))
        ch.setBusOutputControl(self.bitrateFlags)
        ch.setBusParams(self.bitrate)
        ch.busOn()
        return ch

    def receive_from_chassis(self):
        '''**************************************************************************************
        - FunctionName: receive_from_chassis(self)
        - Function    : 接收CAN总线网络数据
        - Inputs      : None
        - Outputs     : None
        - Comments    : 根据DBC筛选接受到的数据
        **************************************************************************************'''
        if rclpy.ok():
            raw_frame = self.ch.read(timeout=100000)
            if raw_frame.id == 0x350:  # WVCU_LongitudinalStatus
                self.wvcu_longitudinal_status_frame_msg = self.wvcu_longitudinal_status_frame.bind(raw_frame)
                self.wvcu_longitudinal_status_msg.wvcu_brk_stoke_stat = float(self.wvcu_longitudinal_status_frame_msg.WVCU_BrkStokeStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_brk_ped_stat = float(self.wvcu_longitudinal_status_frame_msg.WVCU_BrkPedStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_gas_stoke_stat = float(self.wvcu_longitudinal_status_frame_msg.WVCU_GasStokeStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_gas_ped_stat = float(self.wvcu_longitudinal_status_frame_msg.WVCU_GasPedStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_veh_spd = float(self.wvcu_longitudinal_status_frame_msg.WVCU_VehSpd.phys)
                self.wvcu_longitudinal_status_msg.wvcu_lgt_stat_roll_cnt = int(self.wvcu_longitudinal_status_frame_msg.WVCU_LgtStat_RollCnt.phys)
                self.wvcu_longitudinal_status_msg.wvcu_gear_stat = int(self.wvcu_longitudinal_status_frame_msg.WVCU_GearStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_lgt_dsbl_stat = bool(self.wvcu_longitudinal_status_frame_msg.WVCU_LgtDsblStat.phys)
                self.wvcu_longitudinal_status_msg.wvcu_lgt_stat_check_sum = int(self.wvcu_longitudinal_status_frame_msg.WVCU_LgtStat_CheckSum.phys)

                self.publisher_wvcu_longitudinal_status_feedback.publish(self.wvcu_longitudinal_status_msg)
                self.longitudinal_feedback_csv_logger.info("{},{}".format(self.wvcu_longitudinal_status_msg.wvcu_brk_stoke_stat, self.wvcu_longitudinal_status_msg.wvcu_gas_stoke_stat))
                
            elif raw_frame.id == 0x351:  # WVCU_HorizontalStatus
                self.wvcu_horizontal_status_msg_frame_msg = self.wvcu_horizontal_status_frame.bind(raw_frame)
                self.wvcu_horizontal_status_msg.wvcu_str_whl_ang_stat = float(self.wvcu_horizontal_status_msg_frame_msg.WVCU_StrWhlAngStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_str_whl_tq = float(self.wvcu_horizontal_status_msg_frame_msg.WVCU_StrWhlTq.phys)
                self.wvcu_horizontal_status_msg.wvcu_manl_gear_intv = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ManlGearIntv.phys)
                self.wvcu_horizontal_status_msg.wvcu_manl_exit_adu_sw_intv = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ManlExitADUSwIntv.phys)
                self.wvcu_horizontal_status_msg.wvcu_manl_brk_ped_intv = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ManlBrkPedIntv.phys)
                self.wvcu_horizontal_status_msg.wvcu_manl_drv_ped_intv = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ManlDrvPedIntv.phys)
                self.wvcu_horizontal_status_msg.wvcu_shake_stat = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ShakeStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_adu_can_stat = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ADUCANStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_req_shake_sw_stat = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_ReqShakeSwStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_intv_str_whl_stat = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_IntvStrWhlStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_eps_exit_flt = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_EPSExitFlt.phys)
                self.wvcu_horizontal_status_msg.wvcu_hozl_stat_roll_cnt = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_HozlStat_RollCnt.phys)
                self.wvcu_horizontal_status_msg.wvcu_hozl_dsbl_stat = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_HozlDsblStat.phys)
                self.wvcu_horizontal_status_msg.wvcu_hozl_stat_check_sum = int(self.wvcu_horizontal_status_msg_frame_msg.WVCU_HozlStat_CheckSum.phys)

                self.publisher_wvcu_horizontal_status_feedback.publish(self.wvcu_horizontal_status_msg)
                
                self.get_logger().info("Steering angle: %s" % self.eps_feedback_bmsg.StrAng.phys)
                self.eps_feedback_csv_logger.info(self.wvcu_horizontal_status_msg.wvcu_str_whl_ang_stat)  # 记录到csv里面的日志文档 ("{},{}".format(11,12)
                
            elif raw_frame.id == 0x370:  # WVCU_BodyStatus
                self.wvcu_body_status_frame_msg = self.wvcu_body_status_frame.bind(raw_frame)
                self.wvcu_body_status_msg.wvcu_turn_rlamp_stat = int(self.wvcu_body_status_frame_msg.WVCU_TurnRLampStat.phys)
                self.wvcu_body_status_msg.wvcu_turn_llamp_stat = int(self.wvcu_body_status_frame_msg.WVCU_TurnLLampStat.phys)
                self.wvcu_body_status_msg.adu_body_status_roll_cnt = int(self.wvcu_body_status_frame_msg.ADU_BodyStatus_RollCnt.phys)
                self.wvcu_body_status_msg.adu_body_status_check_sum = int(self.wvcu_body_status_frame_msg.ADU_BodyStatus_CheckSum.phys)
                
                self.publisher_wvcu_body_status_feedback.publish(self.wvcu_body_status_msg)

            elif raw_frame.id == 770:  # VCU_Speed_Feedback
                
                self.wvcu_flt_cod_frame_msg = self.wvcu_flt_cod_frame.bind(raw_frame)
                self.wvcu_flt_cod_msg.wvcuehb_brk_req_warn = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_BrkReqWarn.phys)
                self.wvcu_flt_cod_msg.wvcuehb_can_bus_off = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_CANBusOff.phys)
                self.wvcu_flt_cod_msg.wvcuehb_isnsr_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_ISnsrFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_load_mismatch_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_LoadMismatchFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_mot_posn_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_MotPosnFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_ntc_flt_lv_1 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_NTCFltLv1.phys)
                self.wvcu_flt_cod_msg.wvcuehb_ntc_flt_lv_2 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_NTCFltLv2.phys)
                self.wvcu_flt_cod_msg.wvcuehb_over_iflt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_OverIFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_over_temp_warn = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_OverTempWarn.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pctrl_vib_lv1 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PCtrlVibLv1.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pctrl_vib_lv2 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PCtrlVibLv2.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pedl_snsr_flt_both = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PedlSnsrFltBoth.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pedl_snsr_flt_single = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PedlSnsrFltSingle.phys)
                self.wvcu_flt_cod_msg.wvcuehb_p_folw_flt_lv1 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PFolwFltLv1.phys)
                self.wvcu_flt_cod_msg.wvcuehb_p_folw_flt_lv2 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PFolwFltLv2.phys)
                self.wvcu_flt_cod_msg.wvcuehb_p_snsr_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PSnsrFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pwr_drvr_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PwrDrvrFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_pwr_swt_flt = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_PwrSwtFlt.phys)
                self.wvcu_flt_cod_msg.wvcuehb_u_sply_high_lv_1 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_USplyHighLv1.phys)
                self.wvcu_flt_cod_msg.wvcuehb_u_sply_high_lv_2 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_USplyHighLv2.phys)
                self.wvcu_flt_cod_msg.wvcuehb_u_sply_low_lv_1 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_USplyLowLv1.phys)
                self.wvcu_flt_cod_msg.wvcuehb_u_sply_low_lv_2 = int(self.wvcu_flt_cod_frame_msg.WVCUEHB_USplyLowLv2.phys)
                self.wvcu_flt_cod_msg.wvcueps_inhibit_code = int(self.wvcu_flt_cod_frame_msg.WVCUEPS_InhibitCode.phys)
                self.wvcu_flt_cod_msg.wvcu_flt_cod_roll_cnt = int(self.wvcu_flt_cod_frame_msg.WVCU_FltCod_RollCnt.phys)
                self.wvcu_flt_cod_msg.wvcu_flt_cod_check_sum = int(self.wvcu_flt_cod_frame_msg.WVCU_FltCod_CheckSum.phys)

                self.publisher_wvcu_flt_cod_feedback.publish(self.wvcu_flt_cod_msg)
                
                if self.wvcu_flt_cod_msg.wvcuehb_brk_req_warn + self.wvcu_flt_cod_msg.wvcuehb_can_bus_off+ self.wvcu_flt_cod_msg.wvcuehb_isnsr_flt + \
                        self.wvcu_flt_cod_msg.wvcuehb_load_mismatch_flt + self.wvcu_flt_cod_msg.wvcuehb_mot_posn_flt + self.wvcu_flt_cod_msg.wvcuehb_ntc_flt_lv_1 + \
                            self.wvcu_flt_cod_msg.wvcuehb_ntc_flt_lv_2 + self.wvcu_flt_cod_msg.wvcuehb_over_iflt + self.wvcu_flt_cod_msg.wvcuehb_over_temp_warn+\
                                self.wvcu_flt_cod_msg.wvcuehb_pctrl_vib_lv1 + self.wvcu_flt_cod_msg.wvcuehb_pctrl_vib_lv2 + self.wvcu_flt_cod_msg.wvcuehb_pedl_snsr_flt_both + \
                                    self.wvcu_flt_cod_msg.wvcuehb_pedl_snsr_flt_single + self.wvcu_flt_cod_msg.wvcuehb_p_folw_flt_lv1 + self.wvcu_flt_cod_msg.wvcuehb_p_folw_flt_lv2 + \
                                        self.wvcu_flt_cod_msg.wvcuehb_p_snsr_flt + self.wvcu_flt_cod_msg.wvcuehb_pwr_drvr_flt + self.wvcu_flt_cod_msg.wvcuehb_pwr_swt_flt +\
                                            self.wvcu_flt_cod_msg.wvcuehb_u_sply_high_lv_1 + self.wvcu_flt_cod_msg.wvcuehb_u_sply_high_lv_2 + self.wvcu_flt_cod_msg.wvcuehb_u_sply_low_lv_1+\
                                                self.wvcu_flt_cod_msg.wvcuehb_u_sply_low_lv_2 + self.wvcu_flt_cod_msg.wvcueps_inhibit_code + self.wvcu_flt_cod_msg.wvcu_flt_cod_roll_cnt + \
                                                    self.wvcu_flt_cod_msg.wvcu_flt_cod_check_sum > 0:
                    self.get_logger().error("Find chassis error!!!!!!")

            elif raw_frame.id == 0x352:  # WVCU_VehSphLim
                self.wvcu_veh_sph_lim_frame_msg = self.wvcu_veh_sph_lim_frame.bind(raw_frame)
                self.wvcu_veh_sph_lim_msg.wvcu_adu_lim_gas_spd = float(self.wvcu_body_status_frame_msg.WVCU_ADULimGasSpd.phys)
                
                self.publisher_wvcu_veh_sph_lim_feedback.publish(self.wvcu_veh_sph_lim_msg)

    def closeChannel_kavser(self, ch):
        '''**************************************************************************************
        - FunctionName: closeChannel_kavser()
        - Function    : 失能并关闭通
        - Inputs      : 待关闭的通道
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        ch.busOff()
        ch.close()

def main(args=None):
    '''**************************************************************************************
    - FunctionName: 
    - Function    : 
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    rclpy.init(args=args)
    chassis_communication_send = ChassisCommunicationSend()
    rclpy.spin(chassis_communication_send)
    chassis_communication_send.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
