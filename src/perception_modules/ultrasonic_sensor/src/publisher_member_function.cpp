// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <getopt.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "ultrasonic_sensor/ICANCmd.h"

using namespace std::chrono_literals;
#define __countof(a)  (sizeof(a)/sizeof(a[0]))

DWORD dwDeviceHandle;
typedef struct {
   int Run;
   DWORD ch;
}rcv_thread_arg_t;

int reclen = 0;
CAN_DataFrame rec[100];
int i;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_l = this->create_publisher<std_msgs::msg::Int16>("d_l", 10);
      publisher_lm = this->create_publisher<std_msgs::msg::Int16>("d_lm", 10);
      publisher_ls = this->create_publisher<std_msgs::msg::Int16>("d_ls", 10);
      publisher_r = this->create_publisher<std_msgs::msg::Int16>("d_r", 10);
      publisher_rs = this->create_publisher<std_msgs::msg::Int16>("d_rs", 10);
      
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      if((reclen = CAN_ChannelReceive(dwDeviceHandle, 0, rec, __countof(rec), 200))>0)
      {
        auto message_l = std_msgs::msg::Int16();
        auto message_lm = std_msgs::msg::Int16();
        auto message_ls = std_msgs::msg::Int16();
        auto message_r = std_msgs::msg::Int16();
        auto message_rs = std_msgs::msg::Int16();

        if(rec[reclen - 2].uID==0x700)//接收CAN上的超声波数据
        {
          message_r.data = rec[reclen - 2].arryData[7]*2;
          message_lm.data = rec[reclen - 2].arryData[5]*2;
          message_l.data = rec[reclen - 2].arryData[4]*2;
        }
        if(rec[reclen - 1].uID==0x701)//接收CAN上的超声波数据
        {
          message_rs.data = rec[reclen - 1].arryData[0]*2;
          message_ls.data = rec[reclen - 1].arryData[2]*2;
        }
        publisher_l->publish(message_l);
        publisher_lm->publish(message_lm);
        publisher_ls->publish(message_ls);
        publisher_r->publish(message_r);
        publisher_rs->publish(message_rs);
      }
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_l;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_lm;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_ls;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_r;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_rs;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if ( (dwDeviceHandle = CAN_DeviceOpen(2, 0, 0)) == 0 ) {
      printf("open deivce error\n");
      //goto ext;
   }
  // 读取设备信息
  CAN_DeviceInformation DevInfo;
  if ( CAN_GetDeviceInfo(dwDeviceHandle, &DevInfo) != CAN_RESULT_OK ) {
    printf("GetDeviceInfo error\n");
    //goto ext;
  }
  // 启动CAN通道
  CAN_InitConfig config;
  config.dwAccCode = 0;
  config.dwAccMask = 0xffffffff;
  config.nFilter  = 0;       // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
  config.bMode    = 0;       // 工作模式(0表示正常模式,1表示只听模式)
  config.nBtrType = 1;       // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
  config.dwBtr[0] = 0x00;    // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
  config.dwBtr[1] = 0x1c;    // BTR1
  config.dwBtr[2] = 0;
  config.dwBtr[3] = 0;

  if ( CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK ) {
      printf("Start CAN 0 error\n");
      //goto ext;
  }
  // 标准帧唤醒(定制型号支持的可选操作)
  Wakeup_Config  wakeup_config;
  wakeup_config.dwAccCode = (0x123 << 21);  // 帧ID为0x123的标准帧 唤醒
  wakeup_config.dwAccMask = 0x001FFFFF;     // 固定
  wakeup_config.nFilter   = 2;              // 滤波方式(0表示未设置滤波功能-收到任意CAN数据都能唤醒,1表示双滤波,2表示单滤波,3-关闭唤醒功能)
  if ( CAN_SetParam(dwDeviceHandle, 0, PARAM_WAKEUP_CFG, &wakeup_config) != CAN_RESULT_OK ) {
      printf("CAN 0 not support Wakeup\n");
  }
  printf("CAN 0 Wakeup!\n");
  
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
