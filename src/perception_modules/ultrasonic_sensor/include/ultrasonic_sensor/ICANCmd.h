#ifndef _ICANCmd_H_
#define _ICANCmd_H_

#ifdef __cplusplus  
extern "C" { 
#endif

#define TRUE            true
#define FALSE           false
typedef bool            BOOL;
typedef char            CHAR;
typedef unsigned char   UCHAR;
typedef unsigned char   BYTE;
typedef unsigned char   *PBYTE;
typedef unsigned short  WORD;
typedef unsigned short  USHORT;
typedef int             INT;
typedef unsigned int    UINT;
typedef unsigned int    DWORD;
typedef unsigned int    *LPDWORD;
typedef unsigned long        ULONG;
typedef unsigned long long   ULONG64;
typedef void            *PVOID;
typedef void            *LPVOID;
#define __stdcall 
#define PACKED __attribute__( ( packed, aligned(1) ) )

//接口卡类型定义
#define ACUSB_131B              1
#define ACUSB_132B              2
#define ACPCI_251               3
#define ACPCI_252               4
#define ACPCI_254               5

#define ACNET_600               6
#define ACNET_622               7

#define LCPCIE_251              8
#define LCPCIE_252              9

#define LCPCIE_361              10
#define LCPCIE_362              11
#define LCPCIE_364              12

#define LCUSB_131B              1
#define LCUSB_132B              2
#define LCMiniPcie_431          1
#define LCMiniPcie_432          2
#define LCPCI_252               4

#define USBCAN_1CH              13
#define USBCAN_C_1CH            14
#define USBCAN_E_1CH            15
#define USBCAN_E_2CH            16
#define MPCIeCAN_1CH            17
#define MPCIeCAN_2CH            18

//函数调用返回状态值
#define	CAN_RESULT_OK			1
#define CAN_RESULT_ERROR		0

//CAN错误码
enum CAN_ErrorCode
{	
	CAN_E_NOERROR        = 0x0000,   // 没有发现错误
	CAN_E_OVERFLOW       = 0x0001,   // CAN控制器内部FIFO溢出
	CAN_E_ERRORALARM     = 0x0002,   // CAN控制器错误报警
	CAN_E_PASSIVE        = 0x0004,   // CAN控制器消极错误
	CAN_E_LOSE           = 0x0008,   // CAN控制器仲裁丢失
	CAN_E_BUSERROR       = 0x0010,   // CAN控制器总线错误

	CAN_E_RCV_BUF        = 0x0020,   // dll CAN接收BUF出错
	CAN_E_ERR_BUF        = 0x0040,   // dll CAN错误信息BUF出错

	CAN_E_DEVICEOPENED   = 0x0100,	// 设备已经打开
	CAN_E_DEVICEOPEN     = 0x0200,	// 打开设备错误
	CAN_E_DEVICENOTOPEN  = 0x0400,	// 设备没有打开
	CAN_E_BUFFEROVERFLOW = 0x0800,	// 缓冲区溢出
	CAN_E_DEVICENOTEXIST = 0x1000,	// 此设备不存在
	CAN_E_LOADKERNELDLL  = 0x2000,	// 装载动态库失败
	CAN_E_CMDFAILED      = 0x4000,	// 执行命令失败错误码
	CAN_E_BUFFERCREATE   = 0x8000,	// 内存不足
};	

// 唤醒滤波参数
typedef struct tagWakeup_Config
{
	DWORD	dwAccCode;						// 验收码
	DWORD	dwAccMask;						// 屏蔽码
	DWORD	nFilter;						// 滤波方式(0表示未设置滤波功能-接收到任意CAN报文都通过USB唤醒主机,1表示双滤波,2表示单滤波,3-关闭唤醒功能)
}PACKED Wakeup_Config, *PWakeup_Config;

enum ParamType
{
	PARAM_WAKEUP_CFG = 0x4000,      // 设置唤醒配置参数
};

enum libcancmd_log_level {
	_LIBUSB_LOG_LEVEL_NONE    = 0,
	_LOG_LEVEL_INFO           = 2,           // 2
	_LOG_LEVEL_WARNING        = 4,        // 4
	_LOG_LEVEL_ERROR          = 8,          // 8
	_LOG_LEVEL_CALL           = 16,           // 16
	_LIBUSB_LOG_LEVEL_ERROR   = 32,   // 32
	_LIBUSB_LOG_LEVEL_WARNING = 64, // 64
	_LIBUSB_LOG_LEVEL_INFO    = 128,   // 128
	_LIBUSB_LOG_LEVEL_DEBUG   = 256,  // 256
};

typedef void(*libcancmd_log_cb)(enum libcancmd_log_level level, const char *str);


//CAN数据帧类型
typedef  struct  tagCAN_DataFrame {
	UINT	uTimeFlag;                      // 时间标识,对接收帧有效
	BYTE	nSendType;                      // 发送帧类型,0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
	BYTE	bRemoteFlag;                    // 是否是远程帧
	BYTE	bExternFlag;                    // 是否是扩展帧
	BYTE	nDataLen;                       // 数据长度
	UINT	uID;                            // 报文DI
	BYTE	arryData[8];                    // 报文数据
}PACKED CAN_DataFrame, *PCAN_DataFrame;

//CAN初始化配置
typedef struct tagCAN_InitConfig {
	UCHAR	bMode;	                        // 工作模式(0表示正常模式,1表示只听模式)
	BYTE	nBtrType;                       // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
	BYTE    dwBtr[4];						// CAN位定时参数
	DWORD	dwAccCode;						// 验收码
	DWORD	dwAccMask;						// 屏蔽码
	BYTE	nFilter;						// 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
	BYTE    dwReserved;                     // 预留字段
}PACKED CAN_InitConfig, *PCAN_InitConfig;

//CAN设备信息
typedef  struct  tagCAN_DeviceInformation {
	USHORT	uHardWareVersion;               // 硬件版本
	USHORT	uFirmWareVersion;               // 固件版本
	USHORT	uDriverVersion;                 // 驱动版本
	USHORT	uInterfaceVersion;              // 接口库版本
	USHORT	uInterruptNumber;               // 中断号
	BYTE	bChannelNumber;                 // 有几路CAN
	CHAR	szSerialNumber[20];             // 设备序列号
	CHAR	szHardWareType[40];             // 硬件类型
	CHAR    szDescription[20];			    // 设备描述
} PACKED CAN_DeviceInformation, *PCAN_DeviceInformation; 

//CAN错误信息
typedef struct tagCAN_ErrorInformation {
	UINT			uErrorCode;             // 错误类型   
	BYTE			PassiveErrData[3];      // 消极错误数据
	BYTE			ArLostErrData;          // 仲裁错误数据
} PACKED CAN_ErrorInformation, *PCAN_ErrorInformation;


/**/
	 void   CAN_SetLogCb(libcancmd_log_cb cb);
	 void   CAN_SetDebug(unsigned int level);
	// 打开设备
	 DWORD  CAN_DeviceOpen(DWORD dwType, DWORD dwIndex, CHAR *pDescription);
	// 关闭设备
	 DWORD  CAN_DeviceClose(DWORD dwDeviceHandle);

	// 启动CAN
	 DWORD  CAN_ChannelStart(DWORD dwDeviceHandle, DWORD dwChannel, PCAN_InitConfig pInitConfig);
	// 停止CAN
	 DWORD  CAN_ChannelStop(DWORD dwDeviceHandle, DWORD dwChannel);

	// 获取设备信息
	 DWORD  CAN_GetDeviceInfo(DWORD dwDeviceHandle, PCAN_DeviceInformation pInfo);
	// 获取CAN错误信息
	 DWORD  CAN_GetErrorInfo(DWORD dwDeviceHandle, DWORD dwChannel, PCAN_ErrorInformation pErr);

	// 读EEPROM
	 DWORD  CAN_ReadEEPROM(DWORD dwDeviceHandle, WORD dwAddr, PBYTE pBuff, WORD nLen);
	// 写EEPROM
	 DWORD  CAN_WriteEEPROM(DWORD dwDeviceHandle, WORD dwAddr, PBYTE pBuff, WORD nLen);

	// 发送数据
	 ULONG  CAN_ChannelSend(DWORD dwDeviceHandle, DWORD dwChannel, PCAN_DataFrame pSend, ULONG nCount);
	// 从接收缓冲区中读数据
	 ULONG  CAN_ChannelReceive(DWORD dwDeviceHandle, DWORD dwChannel, PCAN_DataFrame pReceive, ULONG nCount, INT nWaitTime = -1);
	// 获取接收缓冲区帧数
	 ULONG  CAN_GetReceiveCount(DWORD dwDeviceHandle, DWORD dwChannel);
	// 清空接收缓冲区
	 DWORD  CAN_ClearReceiveBuffer(DWORD dwDeviceHandle, DWORD dwChannel);

	//读寄存器
	 DWORD  CAN_ReadRegister(DWORD dwDeviceHandle, DWORD dwChannel, DWORD dwAddr, PBYTE pBuff, WORD nLen);
	//写寄存器
	 DWORD  CAN_WriteRegister(DWORD dwDeviceHandle, DWORD dwChannel, DWORD dwAddr, PBYTE pBuff, WORD nLen);

	// 获取参数
	 DWORD  CAN_GetParam(DWORD dwDeviceHandle, DWORD dwChannel, DWORD dwParamType, PVOID pData);
	// 设置参数
	 DWORD  CAN_SetParam(DWORD dwDeviceHandle, DWORD dwChannel, DWORD dwParamType, PVOID pData);

	#ifdef __cplusplus  
}
#endif

#endif //_ICANCmd_H_
