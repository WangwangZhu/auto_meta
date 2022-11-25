# coding=utf-8
'''*****************************************************************************************************
# FileName    : ins_data_collection.py
# FileFunction: 获取INS通过串口发来的数据并且完成解析，将数据存放到字典里，同时，将数据写入到CSV文件
# Comments    :
*****************************************************************************************************'''
import rclpy
from rclpy.node import Node
from . import ins_data_parse_base_class

'''**************************************************************************************
- FunctionName: Main
- Function    : main
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''

def main(args=None):
    rclpy.init(args=args)
    
    ins_data_collection = ins_data_parse_base_class.InsDataCollection()
    ins_data_collection.data_parse_collect()
    
    ins_data_collection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            
            
            
