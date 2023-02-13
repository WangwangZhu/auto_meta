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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/msg_to_can.hpp" 
#include "cpp_pubsub/ICANCmd.h"


using std::placeholders::_1;
#define __countof(a)  (sizeof(a)/sizeof(a[0]))

DWORD dwDeviceHandle;
typedef struct {
   int Run;
   DWORD ch;
}rcv_thread_arg_t;

typedef struct {
   DWORD ch;
   DWORD sndType;   //0 - 正常发送;1 - 单 次 发送;2 - 自发自收;3 - 单 次 自发自收
   DWORD sndFrames; // 每次发送帧数
   DWORD sndTimes;  // 发送次数
}snd_thread_arg_t;

unsigned long CanSendcount[2] = { 0, 0 };


class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::MsgToCan>(
      "to_can", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::MsgToCan & msg) const
  {



    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.speed << "'");
    CAN_DataFrame *send = new CAN_DataFrame[1];
      int times = 1;  //  26000-12h
      int ch = 0;
      unsigned int speed_can=0,acc_can=0,dec_can=0,se_can=0,con_can=0;
      int angle_can=0;      //goto ext;
      for ( int j = 0; j < 1; j++ ) {
      send[j].uID = 0x203;         // ID
      send[j].nSendType = 1;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
      send[j].bRemoteFlag = 0;  // 0-数据帧；1-远程帧
      send[j].bExternFlag = 0;  // 0-标准帧；1-扩展帧
      send[j].nDataLen = 8;     // DLC
      // for ( int i = 0; i < send[j].nDataLen; i++ ) {
      //    send[j].arryData[i] = msg.connect;
      // }
      speed_can = (unsigned int)(msg.speed);
      dec_can = (unsigned int)(msg.dec * 10);
      angle_can = (int)(msg.angle * 100);
      uint8_t	Data = 0x00;
      uint8_t n[8]={msg.connect,msg.forward,msg.back,0,0,msg.buzzer,0,msg.clock};

      Data = n[0]|(n[1]<<1)|(n[2]<<2)|(n[3]<<3)|(n[4]<<4)|(n[5]<<5)|(n[6]<<6)|(n[7]<<7);
      con_can = (unsigned int)(Data);
      send[j].arryData[0] = (con_can)&0x00ff;
      send[j].arryData[1] = (speed_can)&0x00ff;
      send[j].arryData[2] = (speed_can>>8)&0x00ff;
      send[j].arryData[3] = (acc_can)&0x00ff;
      send[j].arryData[4] = (dec_can)&0x00ff;
      send[j].arryData[5] = (angle_can)&0x00ff;   
      send[j].arryData[6] = (angle_can>>8)&0x00ff;
      send[j].arryData[7] = (se_can)&0x00ff;
   }   
   while ( times ) {
      //printf("CAN%d Send %d\r\n", ch, times);
      unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle, ch, send, 1);
      CanSendcount[ch] += sndCnt;
      if ( sndCnt )
         times--;
   }
   delete[] send;
   printf("CAN%d Send Count:%ld end \r\n", ch, CanSendcount[ch]);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::MsgToCan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  printf("test-------sub");
  rclcpp::init(argc, argv);
  if ( (dwDeviceHandle = CAN_DeviceOpen(2, 0, 0)) == 0 ) {
      printf("open deivce error\n");
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
  // if ( CAN_SetParam(dwDeviceHandle, 0, PARAM_WAKEUP_CFG, &wakeup_config) != CAN_RESULT_OK ) {
  //     printf("CAN 0 not support Wakeup\n");
  // }
  printf("CAN 0 Wakeup!\n");
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
