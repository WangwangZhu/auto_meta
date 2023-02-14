# coding=utf-8
'''*****************************************************************************************************
# FileName    : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''
import sys
import csv
import math
import time
import rclpy
import serial
import struct
import collections
from time import sleep
from rclpy.node import Node
from pyproj import CRS
from pyproj import Transformer
from datetime import datetime
from datetime import timedelta

class InsDataCollection(Node):
    '''*****************************************************************************************************
    - Class Name  : InsDataCollection
    - Function    : 录制惯导数据
    - Inputs      : None
    - Outputs     : None
    - Comments    : 原点定义在 大石头那里,
                    origin_utm_x: 沿着经度方向 Between 120°E and 126°E，向右递增
                    origin_utm_y: 沿着纬度的方向 northern hemisphere between equator and 84°N 向北递增
    *****************************************************************************************************'''
    def __init__(self,
                #  device='/dev/ttyS2',
                 device='/dev/ttyUSB0',
                 baud_rate=115200,
                 timeout=0.5,
                 data_storage_folder='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split('/')[2] + '/' + 'auto_meta/src/location_modules/integrated_navigation_system/data_collection/',
                 origin_longitude=121.4415488,
                 origin_latitude=31.0282838,
                 origin_utm_x=351256.08,
                 origin_utm_y=3433779.41,
                 data_map_sub_name = "_ins_data_map.csv",
                 node_name="ins_d_data_parse"):
        super().__init__(node_name)
    
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.data_storage_folder = data_storage_folder
        self.date_now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.data_map_name = self.data_storage_folder + self.date_now + data_map_sub_name
        self.origin_longitude = origin_longitude
        self.origin_latitude = origin_latitude
        self.datetimeformat = "%Y-%m-%d %H:%M:%S.%f"
        self.origin_utm_x = origin_utm_x
        self.origin_utm_y = origin_utm_y
        self.headers = ['gps_time', 'utc_time', 
                        'heading', 'pitch', 'roll', 
                        'heading_radians', 'pitch_radians', 'roll_radians', 
                        'gyroX', 'gyroY', 'gyroZ', 
                        'accX', 'accY', 'accZ', 
                        'latitude', 'longitude', 'altitude',
                        'utm_x_position', 'utm_y_position', 
                        'local_x', 'local_y',
                        'east_speed', 'north_speed', 'vertical_speed', 
                        'vehicle_lat_speed', 'vehicle_lon_speed',
                        'position_type', 'GNSS_info1', 'GNSS_info2']
        
        self.raw_data_from_buffer_one_package = None

        self.serial_ = serial.Serial(self.device, self.baud_rate, timeout=self.timeout)
        self.writer = None
        
        self.crs = CRS.from_epsg(4326) # WGS84 的 epsg 代码
        self.crs_sh = CRS.from_epsg(32651) # UTM 中 51N（上海所在带）的 epsg 代码
        self.transformer_WGS84toUTM = Transformer.from_crs(self.crs, self.crs_sh)
        
        self.epoch_gps = datetime.strptime("1980-01-06 00:00:00.00", self.datetimeformat) # 系统时间是北京时间
        self.epoch_utc = datetime.strptime("1970-01-01 00:00:00.00", self.datetimeformat)
        
        self.time_now = None
        self.time_now_for_gps_week_calculator = datetime.strptime(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), self.datetimeformat)
        self.time_difference = self.time_now_for_gps_week_calculator - self.epoch_gps
        
        self.gps_week = self.time_difference.days // 7
        self.gps_week_difference = timedelta(days=self.gps_week*7, hours=8) # 8 个小时将时间从 UTC 0 推到 UTC 8
        self.gps_week_start_ms = time.mktime(time.strptime(str(self.epoch_gps + self.gps_week_difference), "%Y-%m-%d %H:%M:%S")) * 1000 

        self.origin_zone = "51N"
        self.isorigin_set = True
        ######### ins_data ###########
        self.gps_time = None
        self.utc_time = None
        
        self.heading = None
        self.pitch = None
        self.roll = None
        
        self.heading_radians = None
        self.pitch_radians = None
        self.roll_radians = None
        
        self.gyroX = None
        self.gyroY = None
        self.gyroZ = None
        
        self.accX = None
        self.accY = None
        self.accZ = None
        
        self.latitude = None
        self.longitude = None
        self.altitude = None
        
        self.utm_x_position = None
        self.utm_y_position = None
        
        self.local_x = None
        self.local_y = None
        
        self.east_speed = None
        self.north_speed = None
        self.vertical_speed = None
        
        self.vehicle_lon_speed = None
        self.vehicle_lat_speed = None
        
        self.position_type = None
        
        self.GNSS_info1 = None
        self.GNSS_info2 = None

    def receive_from_buffer(self):    
        '''**************************************************************************************
        - FunctionName: 
        - Function    : 
        - Inputs      : 
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        global data
        while True:
            if self.serial_.read() == b'\xaa':
                if self.serial_.read() == b'\x55':
                    # self.time_now = int(round(time.time() * 1000)) # ms
                    self.time_now = self.get_clock().now().to_msg()
                    data_raw_ = self.serial_.read(135)
                    # self.get_logger().error(str(time.time() * 1000 - self.time_now)) # 统计读取数据包需要花费的时间
                    data_sum = sum(data_raw_)-data_raw_[133]-data_raw_[134]
                    if data_raw_[133] == (data_sum & 0xff):
                        if data_raw_[134] == (data_sum >> 8 & 0xff):
                            self.raw_data_from_buffer_one_package = data_raw_
                            # self.get_logger().info("package check passed ^~^")
                            sleep(0.001)     
                            break
                    else:
                        self.get_logger().warning("package check error ~~~")
                        
    def block_parse(self):    
        '''**************************************************************************************
        - FunctionName: 
        - Function    : 
        - Inputs      : 
        - Outputs     : None
        - Comments    : INS OPVT2AHR 使用的协议类型
        -               惯导在车上安装后，Y轴方向沿着车辆的纵向；X轴沿着车辆的横向，向右为正；Z轴垂直向上
        -               绕Y轴是 Roll 侧倾角；
        -               绕X轴是 Pitch 俯仰角；
        -               绕Z轴是 Heading(Yaw) 横摆角
        **************************************************************************************'''
        data = self.raw_data_from_buffer_one_package
        # 偏移4：因为一个包开头有6个无用数据，头已经在读取函数中舍弃，因此再舍弃掉无用的四个数据
        self.gps_time = self.gps_week_start_ms + struct.unpack('<i', data[4+104:4+104+4])[0] - 18000
        self.utc_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')
        
        # ********************* 姿态角度 *****************
        self.heading = struct.unpack('<H', data[4+0:4+0+2])[0] * 1e-2 # unsigned short(C)(2bytes) ==> integer(python)        
        # 惯导本体：heading 顺时针为正，0度在正北方向(ins-d)（正东）（根据手册，不满足右手系）
        # 惯导解析节点发出的定位信息：航向起始于X轴正方向（正东），绕Z轴逆时针为正（450deg - raw_heading）
        self.heading = 450 - self.heading
        if self.heading > 360:
            self.heading -= 360
        # 惯导本体：pitch 车头向上抬的时候，角度为正(ins-d)（根据手册，满足右手系）
        # 惯导解析节点发出的定位信息：车屁股抬起，俯仰角为正
        self.pitch = -struct.unpack('<h', data[4+2:4+2+2])[0] * 1e-2 # short(C)(2bytes) ==> integer(python)
        self.roll = struct.unpack('<h', data[4+4:4+4+2])[0] * 1e-2 # short(C)(2bytes) ==> integer(python)

        self.heading_radians = self.heading / 57.29578
        self.pitch_radians = self.pitch / 57.29578
        self.roll_radians = self.roll / 57.29578

        # ********************* 角速度 *****************
        # 角速度满足右手坐标系定义的正方向
        # 参考pitch的取反依据
        self.gyroX = -struct.unpack('<i', data[4+6:4+6+4])[0] * 1e-5 # int(C)(4bytes) ==> integer(python) 角速度 deg/s 俯仰角
        self.gyroY = struct.unpack('<i', data[4+10:4+10+4])[0] * 1e-5 # int(C)(4bytes) ==> integer(python) 侧倾角度
        # heading 虽然不满足右手坐标系，但是横摆角速度是满足右手坐标系的
        self.gyroZ = struct.unpack('<i', data[4+14:4+14+4])[0] * 1e-5 # int(C)(4bytes) ==> integer(python)

        # ********************* 线加速度 *****************
        # 正方向沿着惯导本体的坐标系正方向
        # 从车屁股看，惯导本体横向速度向右为正，但是自动驾驶系统使用的横向速度向左为正
        self.accX = -struct.unpack('<i', data[4+18:4+18+4])[0] * 1e-6 * 9.8 # int(C)(4bytes) ==> integer(python) 线加速度 m/s2
        self.accY = struct.unpack('<i', data[4+22:4+22+4])[0] * 1e-6 * 9.8 # int(C)(4bytes) ==> integer(python)
        self.accZ = struct.unpack('<l', data[4+26:4+26+4])[0] * 1e-6 * 9.8 # int(C)(4bytes) ==> integer(python)

        # ********************* 位置信息 *****************
        self.latitude = struct.unpack('<q', data[4+42:4+42+8])[0] * 1e-9 # long long(C)(8bytes) ==> long(python)
        self.longitude = struct.unpack('<q', data[4+50:4+50+8])[0] * 1e-9 # long long(C)(8bytes) ==> long(python)
        self.altitude = struct.unpack('<i', data[4+58:4+58+4])[0] * 1e-3 # int(C)(4bytes) ==> integer(python)
        # 全局路径坐标系 x 轴沿着经度向右， y轴沿着纬度向上
        self.utm_x_position, self.utm_y_position = self.transformer_WGS84toUTM.transform(self.latitude, self.longitude)
        if self.origin_zone == "51N" and self.isorigin_set == True:
            self.local_x = self.utm_x_position - self.origin_utm_x
            self.local_y = self.utm_y_position - self.origin_utm_y

        # ********************* 速度信息 *****************
        self.east_speed = struct.unpack('<i', data[4+62:4+62+4])[0] * 1e-2 # int(C)(4bytes) ==> integer(python)
        self.north_speed = struct.unpack('<i', data[4+66:4+66+4])[0] * 1e-2 # int(C)(4bytes) ==> integer(python)
        self.vertical_speed = struct.unpack('<i', data[4+70:4+70+4])[0] * 1e-2 # int(C)(4bytes) ==> integer(python)

        self.rad_heading = self.heading / 57.29578
        self.vehicle_lon_speed = self.east_speed * math.cos(self.rad_heading) + self.north_speed * math.sin(self.rad_heading)
        # 从车辆屁股看过去，向左为横向速度正方向
        self.vehicle_lat_speed = -self.east_speed * math.sin(self.rad_heading) + self.north_speed * math.cos(self.rad_heading)
        
        # ********************* 其他信息 *****************
        self.position_type = struct.unpack('<B', data[4+113:4+113+1])[0] # unsigned char(C)(1byte) ==> integer
        self.GNSS_info1 = struct.unpack('<B', data[4+108:4+108+1])[0] & 0x0f # unsigned char(C)(1byte) ==> integer
        self.GNSS_info2 = struct.unpack('<B', data[4+109:4+109+1])[0] # unsigned char(C)(1byte) ==> integer
        
        row_data_in_csv = collections.OrderedDict()
        row_data_in_csv = {
            'gps_time': self.gps_time,
            'utc_time': self.utc_time,

            'heading': self.heading,
            'pitch': self.pitch,
            'roll': self.roll,

            'heading_radians': self.heading_radians,
            'pitch_radians': self.pitch_radians,
            'roll_radians': self.roll_radians,

            'gyroX': self.gyroX,
            'gyroY': self.gyroY,
            'gyroZ': self.gyroZ,
            
            'accX': self.accX,
            'accY': self.accY,
            'accZ': self.accZ,
            
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            
            'utm_x_position': self.utm_x_position,
            'utm_y_position': self.utm_y_position,
            
            'local_x' : self.local_x, 
            'local_y' : self.local_y,
            
            'east_speed': self.east_speed,
            'north_speed': self.north_speed,
            'vertical_speed' : self.vertical_speed,
            
            'vehicle_lat_speed': self.vehicle_lat_speed,
            'vehicle_lon_speed': self.vehicle_lon_speed,
            
            'position_type': self.position_type,
            
            'GNSS_info1': self.GNSS_info1,
            'GNSS_info2': self.GNSS_info2,
            }

        return row_data_in_csv
        
    '''**************************************************************************************
    - FunctionName: 
    - Function    : 
    - Inputs      : 
    - Outputs     : None
    - Comments    : INS OPVT2AHR 使用的协议类型
    -               惯导在车上安装后，Y轴方向沿着车辆的纵向；X轴沿着车辆的横向，向右为正；Z轴垂直向上
    -               绕Y轴是 Roll 侧倾角；
    -               绕X轴是 Pitch 俯仰角；
    -               绕Z轴是 Heading(Yaw) 横摆角
    **************************************************************************************'''    
    def data_parse_collect(self):
        try:
            self.serial_.isOpen()
            # self.serial_.flushInput() # 清理缓冲区内的历史数据
        except :
            self.get_logger().fatal("OPEN INS_D FAILED!!!")
            rclpy.shutdown()
        else:
            self.get_logger().info("ins working")
            with open(self.data_map_name, 'w', newline='') as f:
                self.writer = csv.DictWriter(f, self.headers)
                self.writer.writeheader()   
                while rclpy.ok():
                    self.receive_from_buffer() 
                    data = self.block_parse()
                    self.writer.writerow(data)
                    self.get_logger().info("ins collecting...")