/*
 *  RPLIDAR STM32 Driver
 * 
 *  Copyright (c) Arch-Ci
 *  Author: Arch-Ci
 *  Date: 2019.8
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef RPLIDAR_H_
#define RPLIDAR_H_   
#include "uart_dma.h"

#define      RPLIDAR_CMD_SYNC_BYTE                           0xA5            //雷达命令帧起始字节
//#define      RPLIDAR_CMDFLAG_HAS_PAYLOAD                     0x80

#define      RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1             0xA
#define      RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2             0x5
#define      RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT            (0x1<<15)
#define      RPLIDAR_RESP_MEASUREMENT_SYNCBIT                (0x1<<0)
#define      RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT          2
#define      RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT            1

#define      MAX_SCAN_NODES                                  2048            //最大scan nodes数量 

#define      MAX_MOTOR_PWM                                   39              //最大PWM值
#define      DEFAULT_MOTOR_PWM                               20              //默认PWM值

#define      MAX_MOTOR_RPM                                   900             //最大RPM值
#define      DEFAULT_MOTOR_RPM                               600             //默认RPM值

#define      RPLIDAR_ANS_SYNC_BYTE1                          0xA5            //雷达响应帧起始字节1
#define      RPLIDAR_ANS_SYNC_BYTE2                          0x5A            //雷达响应帧起始字节2
           
#define      RPLIDAR_CMD_GET_ACC_BOARD_FLAG                  0xFF            //询问雷达转接板是否支持motor control
#define      RPLIDAR_CMD_SET_MOTOR_PWM                       0xF0            //向USB转接班发送设置PWM命令

#define      RPLIDAR_CMD_STOP                                0x25            //STOP命令
#define      RPLIDAR_CMD_SCAN                                0x20            //SCAN命令
#define      RPLIDAR_CMD_FORCE_SCAN                          0x21            //FORCE_SCAN命令
#define      RPLIDAR_CMD_RESET                               0x40            //RESET命令
#define      RPLIDAR_CMD_GET_DEVICE_INFO                     0x50            //GET_INFO命令
#define      RPLIDAR_CMD_GET_DEVICE_HEALTH                   0x52            //GET_HEALTH命令
#define      RPLIDAR_CMD_GET_SAMPLERATE                      0x59            //GET_SAMPLERATE命令，added in fw 1.17
#define      RPLIDAR_CMD_EXPRESS_SCAN                        0x82            //EXPRESS_SCAN命令，added in fw 1.17 
//#define      RPLIDAR_CMD_HQ_SCAN                             0x83 

#define      RPLIDAR_CMD_GET_LIDAR_CONF                      0x84            //GET_LIDAR_CONF命令
//#define      RPLIDAR_CMD_SET_LIDAR_CONF                      0x85

#define      RPLIDAR_ANS_TYPE_DEVINFO                        0x04            //查询雷达设备信息的时候，雷达响应帧首部的type会填充此值
#define      RPLIDAR_ANS_TYPE_DEVHEALTH                      0x06            //查询雷达健康信息的时候，雷达响应帧首部的type会填充此值
#define      RPLIDAR_ANS_TYPE_SAMPLE_RATE                    0x15            //查询雷达采样频率的时候，雷达响应帧首部的type会填充此值，added in FW ver 1.17
#define      RPLIDAR_ANS_TYPE_GET_LIDAR_CONF                 0x20            //查询雷达配置信息的时候，雷达响应帧首部的type会填充此值
#define      RPLIDAR_ANS_TYPE_SET_LIDAR_CONF                 0x21
#define      RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG                 0xFF            //询问雷达转接板是否支持motor control的时候，雷达响应帧首部的type会填充此值 

#define      RPLIDAR_ANS_TYPE_MEASUREMENT                    0x81
#define      RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED           0x82            //added in FW ver 1.17
#define      RPLIDAR_ANS_TYPE_MEASUREMENT_HQ                 0x83            //added in FW ver 1.17
#define      RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA     0x84            //added in FW ver 1.23alpha
#define      RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED     0x85            //added in FW ver 1.24


#define      RPLIDAR_CONF_SCAN_COMMAND_STD                   0
#define      RPLIDAR_CONF_SCAN_COMMAND_EXPRESS               1
#define      RPLIDAR_CONF_SCAN_COMMAND_HQ                    2
#define      RPLIDAR_CONF_SCAN_COMMAND_BOOST                 3
#define      RPLIDAR_CONF_SCAN_COMMAND_STABILITY             4
#define      RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY           5
#define      RPLIDAR_CONF_SCAN_COMMAND_DENSEBOOST            6
#define      RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID             0xFF

#define      RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID             0xFF

#define      RPLIDAR_CONF_ANGLE_RANGE                        0x00000000
#define      RPLIDAR_CONF_DESIRED_ROT_FREQ                   0x00000001
#define      RPLIDAR_CONF_SCAN_COMMAND_BITMAP                0x00000002
#define      RPLIDAR_CONF_MIN_ROT_FREQ                       0x00000004
#define      RPLIDAR_CONF_MAX_ROT_FREQ                       0x00000005
#define      RPLIDAR_CONF_MAX_DISTANCE                       0x00000060

        
#define      RPLIDAR_CONF_SCAN_MODE_COUNT                    0x00000070     //获取设备所支持的扫描工作模式ID最大值
#define      RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE            0x00000071     //获取给定扫描工作模式下的采样频率
#define      RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE             0x00000074     //获取给定扫描工作模式下的最大测距半径
#define      RPLIDAR_CONF_SCAN_MODE_ANS_TYPE                 0x00000075     //获取给定扫描工作模式下EXPRESS_SCAN命令请求采用的协议版本
#define      RPLIDAR_CONF_SCAN_MODE_TYPICAL                  0x0000007C     //获取当前设备推荐的扫描工作模式ID
#define      RPLIDAR_CONF_SCAN_MODE_NAME                     0x0000007F     //获取给定扫描工作模式所对应的可供阅读的名称

#define      RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL                 0xA8           //请求设置RPLIDAR测距核心的旋转速度(Rpm)

#define      RPLIDAR_EXPRESS_SCAN_STABILITY_BITMAP           4
#define      RPLIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP         5

#define      RPLIDAR_ANS_HEADER_SIZE_MASK                              0x3FFFFFFF  //可用于取出小端数据（4字节）的低30位，主要用于取出雷达响应帧首部的"数据应答报文长度"；
#define      RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK       0x01        //可用于取出小端数据的最低位，主要用判断雷达转结板是否支持motor control；

// Definition of the variable bit scale encoding mechanism
#define      RPLIDAR_VARBITSCALE_X2_SRC_BIT                  9
#define      RPLIDAR_VARBITSCALE_X4_SRC_BIT                  11
#define      RPLIDAR_VARBITSCALE_X8_SRC_BIT                  12
#define      RPLIDAR_VARBITSCALE_X16_SRC_BIT                 14

#define      RPLIDAR_VARBITSCALE_X2_DEST_VAL                 512
#define      RPLIDAR_VARBITSCALE_X4_DEST_VAL                 1280
#define      RPLIDAR_VARBITSCALE_X8_DEST_VAL                 1792
#define      RPLIDAR_VARBITSCALE_X16_DEST_VAL                3328

#define      DEFAULT_SCAN_MODE_COUNT                         1              //默认的扫描模式数量
#define      MAX_SCAN_MODE_COUNT                             9              //最大的扫描模式数量
#define      DEFAULT_TYPICAL_SCAN_MODE_ID                    0              //默认的推荐扫描模式的ID

typedef void(*_CACHE_TASK)(void* arg); 


/*枚举雷达当前所处状态*/
typedef enum
{
    IS_STOPPED = 0,
    IS_RESET,
    IS_SCANNING,  
    IS_ERROR,
} enum_lidar_status;

/*扫描模式详细信息结构体*/
typedef __packed struct _RplidarScanMode 
{
    unsigned short   id;
    float            us_per_sample;      // microseconds per sample
    float            max_distance;       // max distance
    unsigned char    ans_type;           // the answer type of the scam mode, it's value should be RPLIDAR_ANS_TYPE_MEASUREMENT*
    char             scan_mode[64];      // name of scan mode, max 63 characters
} RplidarScanMode;

/*雷达命令帧首部*/
typedef __packed struct _rplidar_cmd_header_t 
{
    unsigned char    syncByte; 
    unsigned char    cmdByte; 
} rplidar_cmd_header_t;

/*雷达Express命令帧负载*/
typedef __packed struct _rplidar_payload_express_scan_t 
{
    unsigned char    working_mode;
    unsigned short   working_flags;
    unsigned short   param;
} rplidar_payload_express_scan_t;

/*查询雷达是否使用USB转结板命令帧主体*/
typedef __packed struct _rplidar_payload_acc_board_flag_t 
{
    unsigned int     reserved;
} rplidar_payload_acc_board_flag_t;

/*雷达响应是否使用USB转结板命令帧主体*/
typedef __packed struct _rplidar_response_acc_board_flag_t 
{
    unsigned int     support_flag;
} rplidar_response_acc_board_flag_t;

/*雷达响应帧首部*/
typedef __packed struct _rplidar_ans_header_t 
{
    unsigned char    syncByte1;        // must be RPLIDAR_ANS_SYNC_BYTE1
    unsigned char    syncByte2;        // must be RPLIDAR_ANS_SYNC_BYTE2
    unsigned int     size_q30_subtype; // see unsigned int size:30; unsigned int subType:2;
    unsigned char    type;
} rplidar_ans_header_t;

/*雷达响应设备信息帧主体*/
typedef __packed struct _rplidar_response_device_info_t 
{
    unsigned char    model;
    unsigned char    firmware_version_minor;
    unsigned char    firmware_version_major;
    unsigned char    hardware_version;
    unsigned char    serialnum[16];
} rplidar_response_device_info_t;

/*雷达响应设备健康信息帧主体*/
typedef __packed struct _rplidar_response_device_health_t 
{
    unsigned char    status;
    unsigned short   error_code;
} rplidar_response_device_health_t;

/*雷达响应获取采样频率帧主体，反馈的数据是单次激光测距用时，外部系统可基于这些数据精确计算RPLIDAR的当前采样频率*/
typedef __packed struct _rplidar_response_sample_rate_t 
{
    unsigned short   std_sample_duration_us;
    unsigned short   express_sample_duration_us;
} rplidar_response_sample_rate_t;

/*设置转接板PWM时帧主体*/
typedef __packed struct _rplidar_payload_motor_pwm_t 
{
    unsigned short   pwm_value;
} rplidar_payload_motor_pwm_t;

/*查询雷达扫描配置信息时帧主体*/
typedef __packed struct _rplidar_payload_get_scan_conf_t 
{
    unsigned int     type;
    unsigned char    reserved[32];
} rplidar_payload_get_scan_conf_t;

/*雷达反馈measurement_node*/
typedef __packed struct _rplidar_response_measurement_node_t 
{
    unsigned char    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    unsigned short   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    unsigned short   distance_q2;
} rplidar_response_measurement_node_t;

typedef __packed struct _rplidar_response_cabin_nodes_t 
{
    unsigned short   distance_angle_1;  // see [distance_sync flags]
    unsigned short   distance_angle_2;  // see [distance_sync flags]
    unsigned char    offset_angles_q3;  
} rplidar_response_cabin_nodes_t;   

/*雷达反馈capsule_measurement_nodes*/
typedef __packed struct _rplidar_response_capsule_measurement_nodes_t 
{
    unsigned char           
      s_checksum_1; // see [s_checksum_1]
    unsigned char                   s_checksum_2; // see [s_checksum_1]
    unsigned short                  start_angle_sync_q6;
    rplidar_response_cabin_nodes_t  cabins[16];
} rplidar_response_capsule_measurement_nodes_t;

typedef __packed struct _rplidar_response_measurement_node_hq_t 
{
    unsigned short                  angle_z_q14; 
    unsigned int                    dist_mm_q2; 
    unsigned char                   quality;  
    unsigned char                   flag;
} rplidar_response_measurement_node_hq_t;

/*雷达反馈hq_capsule_measurement_nodes*/
typedef __packed struct _rplidar_response_hq_capsule_measurement_nodes_t
{
    unsigned char                           sync_byte;
    unsigned long long                      time_stamp;
    rplidar_response_measurement_node_hq_t  node_hq[16];
    unsigned int                            crc32;
} rplidar_response_hq_capsule_measurement_nodes_t;

typedef __packed struct _rplidar_response_ultra_cabin_nodes_t 
{
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    unsigned int combined_x3;
} rplidar_response_ultra_cabin_nodes_t;  

/*雷达反馈ultra_capsule_measurement_nodes*/
typedef __packed struct _rplidar_response_ultra_capsule_measurement_nodes_t 
{
    unsigned char                         s_checksum_1; // see [s_checksum_1]
    unsigned char                         s_checksum_2; // see [s_checksum_1]
    unsigned short                        start_angle_sync_q6;
    rplidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
} rplidar_response_ultra_capsule_measurement_nodes_t;

typedef __packed struct _rplidar_response_dense_cabin_nodes_t 
{
    unsigned short   distance; 
} rplidar_response_dense_cabin_nodes_t;

/*雷达反馈dense_capsule_measurement_nodes*/
typedef __packed struct _rplidar_response_dense_capsule_measurement_nodes_t 
{
    unsigned char                         s_checksum_1; // see [s_checksum_1]
    unsigned char                         s_checksum_2; // see [s_checksum_1]
    unsigned short                        start_angle_sync_q6;
    rplidar_response_dense_cabin_nodes_t  cabins[40];
} rplidar_response_dense_capsule_measurement_nodes_t;


/*雷达驱动结构体*/
typedef struct _rplidar_drv 
{
    bool               isConnected;                                   /*表征是否已和雷达连接*/
    bool               isSupportingMotorCtrl;                         /*表征转接板是否已和支持motor control*/
    enum_lidar_status  currentStatus;                                 /*表征雷达当前所处的状态*/
    unsigned int       currentScanModeID;                             /*表征当前启动的扫描模式的ID*/
    unsigned int       currentScanMode;                               /*表征当前启动的扫描模式*/   
   
    unsigned int       current_PWM;                                   /*当前设置的PWM值（适用于A系列雷达）*/
    unsigned int       current_RPM;                                   /*当前设置的RPM值（适用于S、T系列雷达）*/

    rplidar_response_device_info_t         lidar_info;                /*雷达设备信息*/
    rplidar_response_device_health_t       health_info;               /*雷达健康信息*/
    rplidar_response_sample_rate_t         rate_info;                 /*雷达采样频率信息*/    

    unsigned short     ScanModeCount;                                 /*雷达支持的扫描模式数量，默认值应当为DEFAULT_SCAN_MODE_COUNT*/
    unsigned short     TypicalScanModeID;                             /*雷达推荐的扫描模式的ID，默认值应当为DEFAULT_TYPICAL_SCAN_MODE_ID*/
    RplidarScanMode    ScanMode[MAX_SCAN_MODE_COUNT];                 /*雷达支持的扫描模式的详细信息，默认应当具有2种扫描模式的信息*/

    _CACHE_TASK        current_cache_proc;                            /*当前解析雷达数据的函数指针*/

    enum_buffer_state  bufferState_A;
    enum_buffer_state  bufferState_B;
    unsigned int       validNodePos_A;
    unsigned int       validNodePos_B;
    unsigned int       ready_time_ms_A;
    unsigned int       ready_time_ms_B;
    rplidar_response_measurement_node_hq_t  nodeBuffer_A[MAX_SCAN_NODES];
    rplidar_response_measurement_node_hq_t  nodeBuffer_B[MAX_SCAN_NODES];

    bool (*getDeviceInfo) (unsigned int time_out_ms);             /*获取雷达设备信息*/
    bool (*getHealth)     (unsigned int time_out_ms);             /*查询雷达健康信息*/
    bool (*checkMotorCtrlSupport) (unsigned int time_out_ms);     /*询问雷达转接板是否支持motor control*/
    //bool (*getAllSupportedScanModes) (RplidarScanMode **scan_mode_array, unsigned short *scan_mode_count, unsigned int time_out_ms);/*获取所有支持的扫描模式的详细信息*/
    bool (*getAllSupportedScanModes) (unsigned int time_out_ms);  /*获取所有支持的扫描模式的详细信息*/

    bool (*startScan)  (bool force_scan_flag, unsigned short scan_mode_id, unsigned int time_out_ms);  /*开始扫描*/
    bool (*stopScan)   (void);                                     /*停止扫描*/
    bool (*resetLidar) (void);                                     /*重置雷达*/
    bool (*startMotor) (void);                                     /*启动电机*/
    bool (*stopMotor)  (void);                                     /*停止电机*/
    bool (*setMotorPWM) (unsigned short pwm);                      /*设定外部PWM*/
    bool (*setMotorRpm) (unsigned short rpm);                      /*请求设置RPLIDAR测距核心的旋转速度(Rpm)*/
    bool (*grabScanDataHq) (rplidar_response_measurement_node_hq_t *nodebuffer, unsigned int *count, unsigned int time_out_ms);/*获取标准格式的雷达数据*/

    /*以下函数需要用户构造并提供*/
    void (*clearRxBuffer) (void);
    void (*clearTxBuffer) (void);
    int  (*sendData) (char *addr, int count);
    int  (*recvData) (char *addr, int count, UART1_RX_BUFFER **src_addr);
    bool (*waitData) (int wait_size, unsigned int *time_out_ms, UART1_RX_BUFFER **src_addr);
    bool (*createTask) (_CACHE_TASK proc, void* arg);
    bool (*deleteTask) (_CACHE_TASK proc);	
} rplidar_drv;

extern rplidar_drv LidarDrv;

void   LidarDrvInit  (void);                                           /*雷达驱动初始化*/
void   LidarInfoInit (void);                                           /*雷达相关信息初始化*/
void   LidarFuncInit (void);                                           /*雷达相关函数初始化*/
void   LidarSerialFuncInit (void);                                     /*雷达串口相关函数初始化*/
void   LidarTcpFuncInit (void);                                        /*雷达TCP相关函数初始化*/

static bool sendLidarCommand (unsigned char cmd, const void *payload, unsigned char payloadsize);  /*向雷达发送命令*/
static bool getLidarConf(unsigned int cmd_type, char *reserved_addr, unsigned int reserved_size, char **ret_addr, unsigned int *ret_size, unsigned int time_out_ms);    /*查询雷达配置信息*/
static bool getScanModeCount (unsigned int time_out_ms);           /*获取支持的扫描模式数量*/
static bool getTypicalScanMode (unsigned short *typical_mode_id, unsigned int time_out_ms);        /*获取雷达推荐的扫描工作模式的ID*/

static bool checkLidarConnection (unsigned int time_out_ms);       /*检查雷达连接状态*/
static unsigned short identifyScanMode (unsigned short scan_mode_id);  /*识别传入的扫描模式*/
static void cacheScanData(void* arg);                                  /*获取并解析ScanData*/
static void cacheHqScanData(void* arg);                                /*获取并解析HqScanData*/
static void cacheCapsuledScanData(void* arg);                          /*获取并解析CapsuledScanData*/
static void cacheUltraCapsuledScanData(void* arg);                     /*获取并解析UltraCapsuledScanData*/
static void capsuleToNormal(rplidar_response_capsule_measurement_nodes_t *previous_capsule, const rplidar_response_capsule_measurement_nodes_t *capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count);                                                          /*转换CapsuledScanData到HQ格式，并存入hq_buf*/  
 
static void ultraCapsuleToNormal(rplidar_response_ultra_capsule_measurement_nodes_t *previous_ultra_capsule, const rplidar_response_ultra_capsule_measurement_nodes_t *ultra_capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count);      /*转换UltraCapsuledScanData到HQ格式，并存入hq_buf*/

static void dense_capsuleToNormal (rplidar_response_capsule_measurement_nodes_t *previous_capsule, const rplidar_response_capsule_measurement_nodes_t *capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count);                                             /*转换DenseCapsuledScanData到HQ格式，并存入hq_buf*/          

static bool _getDeviceInfo (unsigned int time_out_ms);             /*获取雷达设备信息*/
static bool _getHealth(unsigned int time_out_ms);                  /*查询雷达健康信息*/
static bool _checkMotorCtrlSupport (unsigned int time_out_ms);     /*询问雷达转接板是否支持motor control*/
//static bool _getAllSupportedScanModes (RplidarScanMode **scan_mode_array, unsigned short *scan_mode_count, unsigned int time_out_ms);/*获取所有支持的扫描模式的详细信息*/
static bool _getAllSupportedScanModes (unsigned int time_out_ms);  /*获取所有支持的扫描模式的详细信息*/

static bool _startScan  (bool force_scan_flag, unsigned short scan_mode_id, unsigned int time_out_ms);  /*开始扫描*/
static bool _stopScan   (void);                                    /*停止扫描*/
static bool _resetLidar (void);                                    /*重置雷达*/
static bool _startMotor (void);                                    /*启动电机*/
static bool _stopMotor  (void);                                    /*停止电机*/
static bool _setMotorPWM (unsigned short pwm);                     /*设定外部PWM*/
static bool _setMotorRpm (unsigned short rpm);                     /*请求设置RPLIDAR测距核心的旋转速度(Rpm)*/
static bool _grabScanDataHq (rplidar_response_measurement_node_hq_t *nodebuffer, unsigned int *count, unsigned int time_out_ms);/*获取标准格式的雷达数据*/

static void notify_data_ready_event (void* arg);                       /*通知一圈数据已经准备好*/
#endif


