# coding=utf-8
'''**************************************************************************************
- FileName    : None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''
import sys
import os
path_ = sys.argv[0].split('/')
abs_path_ = '/' + path_[1] + '/' + path_[2] + '/anaconda3/lib/python3.8/site-packages/'
sys.path.append(abs_path_)
import math
import numpy as np
import pandas as pd
from pandas import read_csv
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import rclpy
from rclpy.node import Node
import sys
from collections import OrderedDict
import collections

class MapPreprocess(Node):
    '''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''

    def __init__(self,
                 raw_data_folder='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split(
                     '/')[2] + '/' + 'auto_meta/src/location_modules/integrated_navigation_system/data_collection/',
                 raw_data_file_name='2023_02_01_09_58_16_ins_data_map.csv',
                 data_storage_folder='/' + sys.argv[0].split('/')[1] + '/' + sys.argv[0].split(
                     '/')[2] + '/' + 'auto_meta/src/location_modules/integrated_navigation_system/map_after_preprocess/',
                 data_map_sub_name="_after_preprocess.csv",
                 ):
        '''**************************************************************************************
        - FunctionName:
        - Function    :
        - Inputs      :
        - Outputs     : None
        - Comments    : INS OPVT2AHR 使用的协议类型
        **************************************************************************************'''
        super().__init__('map_preprocess')
        self.data_storage_folder = data_storage_folder
        self.date_now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.data_map_name = self.data_storage_folder + raw_data_file_name
        self.data_raw_map_name = raw_data_folder + raw_data_file_name
        self.raw_data = np.array(pd.read_csv(self.data_raw_map_name))
        
        self.local_x = self.raw_data[:, 19]
        self.local_y = self.raw_data[:, 20]
        
        self.heading = self.raw_data[:, 2]
        
        self.latitude = self.raw_data[:, 14]
        self.longitude = self.raw_data[:, 15]
        self.altitude = self.raw_data[:, 16]
        
        self.raw_index = 1
        self.ref_index = 0
        self.utm_x_y_ref_position = [self.raw_index, self.heading[0], self.local_x[0], self.local_y[0], self.longitude[0], self.latitude[0], self.altitude[0]]
        self.dis_threshold = 6  # 单位是m 距离差值的阈值
        self.headers = ['index', 'heading', 'local_x', 'local_y', 'longitude', 'latitude', 'altitude', "s"]
        
    def cal_absolute_distance(self, utm_x_ref, utm_y_ref, utm_x_raw, utm_y_raw):
        '''**************************************************************************************
        - FunctionName: None
        - Function    : None
        - Inputs      : None
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        return math.sqrt((utm_x_ref-utm_x_raw)**2 + (utm_y_ref-utm_y_raw)**2)
    
    def preprocess(self):
        '''**************************************************************************************
        - FunctionName: None
        - Function    : None
        - Inputs      : None
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        # self.utm_x_y_position.append(self.utm_x_y_ref_position)
        s = 0
        with open(self.data_map_name, 'w', newline='') as f:
            self.writer = csv.DictWriter(f, self.headers)
            
            self.writer.writeheader()  
            
            # 将第一个点先写入
            row_data_in_csv = collections.OrderedDict()
            row_data_in_csv = {
                'index':self.utm_x_y_ref_position[0],
                'heading':self.utm_x_y_ref_position[1],
                'local_x': round(self.utm_x_y_ref_position[2], 3),
                'local_y': round(self.utm_x_y_ref_position[3], 3),
                'longitude': self.utm_x_y_ref_position[4],
                'latitude': self.utm_x_y_ref_position[5],
                'altitude': self.utm_x_y_ref_position[6],
                's': s,
                }
            
            self.writer.writerow(row_data_in_csv)
            
            while self.raw_index < len(self.raw_data):
                _distance = self.cal_absolute_distance(self.local_x[self.ref_index], self.local_y[self.ref_index],self.local_x[self.raw_index], self.local_y[self.raw_index])
                
                if _distance < self.dis_threshold:
                    self.raw_index = self.raw_index + 1
                    # self.get_logger().info("parseing", self.ref_index)
                else:
                    print(_distance)
                    s += _distance
                    self.get_logger().info("parseing"+str(self.ref_index))
                    row_data_in_csv = collections.OrderedDict()
                    row_data_in_csv = {
                        'index':self.raw_index,
                        'heading':self.heading[self.raw_index],
                        'local_x': round(self.local_x[self.raw_index], 3),
                        'local_y': round(self.local_y[self.raw_index], 3),
                        'longitude': self.longitude[self.raw_index],
                        'latitude': self.latitude[self.raw_index],
                        'altitude': self.altitude[self.raw_index],
                        "s": s
                        }
                    self.writer.writerow(row_data_in_csv)
                    
                    self.ref_index = self.raw_index
                    
    def plot_raw_latitude_longitude(self):
        '''**************************************************************************************
        - FunctionName: None
        - Function    : None
        - Inputs      : None
        - Outputs     : None
        - Comments    : None
        **************************************************************************************'''
        minLat = min(self.latitude)
        maxLat = max(self.latitude)
        minLon = min(self.longitude)
        maxLon = max(self.longitude)
        plt.figure(num = '经纬度')
        plt.title('Map')
        plt.scatter(self.latitude,self.longitude)
        plt.xlim(minLat,maxLat)
        plt.ylim(minLon,maxLon)
        plt.xticks(np.arange(minLat,maxLat,0.0003))
        plt.yticks(np.arange(minLon,maxLon,0.0003))
        plt.show()
        
    def plot_utm_x_y(self):
        with open(self.data_map_name, 'r') as f:
            self.raw_data = np.array(pd.read_csv(f))
            self.local_x = self.raw_data[:, 2]
            self.local_y = self.raw_data[:, 3]  
            
            minLon = min(self.local_x)
            maxLon = max(self.local_x)
            minLat = min(self.local_y)
            maxLat = max(self.local_y) 
            plt.scatter(self.local_y, self.local_x)
            plt.xlim(minLat,maxLat)
            plt.ylim(minLon,maxLon)
            plt.xticks(np.arange(minLat,maxLat,30))
            plt.yticks(np.arange(minLon,maxLon,30))
            self.get_logger().info('plotting')
            
            plt.show()
            
def main(args=None):
    '''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    rclpy.init(args=args)

    map_preprocess = MapPreprocess()
    map_preprocess.preprocess()
    map_preprocess.plot_utm_x_y()
    map_preprocess.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    '''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''
    main()
