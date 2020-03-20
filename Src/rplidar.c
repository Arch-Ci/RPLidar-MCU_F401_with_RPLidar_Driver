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

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "rplidar.h"
#include "task.h"
#include "serial.h"
#include "net_socket.h"
#include "timer.h"

rplidar_drv LidarDrv;

/*雷达驱动初始化*/
void LidarDrvInit (void)
{
    LidarInfoInit ();
    LidarFuncInit ();
    LidarSerialFuncInit ();
}

/*雷达相关信息初始化*/
void LidarInfoInit (void)
{
    LidarDrv.isConnected = FALSE;
    LidarDrv.isSupportingMotorCtrl = FALSE;
    LidarDrv.currentStatus = IS_STOPPED;
    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
    LidarDrv.current_PWM = DEFAULT_MOTOR_PWM;
    LidarDrv.current_RPM = DEFAULT_MOTOR_RPM;
    
    memset ((char*)&(LidarDrv.lidar_info), 0, sizeof(rplidar_response_device_info_t));
    memset ((char*)&(LidarDrv.health_info), 0, sizeof(rplidar_response_device_health_t));
    memset ((char*)&(LidarDrv.rate_info), 0, sizeof(rplidar_response_sample_rate_t));
    
    LidarDrv.ScanModeCount = DEFAULT_SCAN_MODE_COUNT;
    LidarDrv.TypicalScanModeID = DEFAULT_TYPICAL_SCAN_MODE_ID;
    LidarDrv.ScanMode[0].id = RPLIDAR_CONF_SCAN_COMMAND_STD;
    LidarDrv.ScanMode[0].us_per_sample = 500.0;
    LidarDrv.ScanMode[0].max_distance = 6.0;
    LidarDrv.ScanMode[0].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
    strcpy (LidarDrv.ScanMode[0].scan_mode, "Standard");

    LidarDrv.current_cache_proc = NULL;

    LidarDrv.bufferState_A = IS_EMPTY;
    LidarDrv.bufferState_B = IS_EMPTY;
    LidarDrv.validNodePos_A = 0;
    LidarDrv.validNodePos_B = 0;
    memset((char*)LidarDrv.nodeBuffer_A, 0, (MAX_SCAN_NODES * sizeof(rplidar_response_measurement_node_hq_t)));
    memset((char*)LidarDrv.nodeBuffer_B, 0, (MAX_SCAN_NODES * sizeof(rplidar_response_measurement_node_hq_t)));
}

/*雷达相关函数初始化*/
void LidarFuncInit (void)
{
    LidarDrv.getDeviceInfo = _getDeviceInfo;
    LidarDrv.getHealth = _getHealth;
    LidarDrv.checkMotorCtrlSupport = _checkMotorCtrlSupport;
    LidarDrv.getAllSupportedScanModes = _getAllSupportedScanModes;

    LidarDrv.startScan = _startScan;
    LidarDrv.stopScan = _stopScan;
    LidarDrv.resetLidar = _resetLidar;
    LidarDrv.startMotor = _startMotor;
    LidarDrv.stopMotor = _stopMotor;
    LidarDrv.setMotorPWM = _setMotorPWM;
    LidarDrv.setMotorRpm = _setMotorRpm;
    LidarDrv.grabScanDataHq = _grabScanDataHq;
}

/*雷达串口相关函数初始化*/
void LidarSerialFuncInit (void)
{
    /*以下函数需要用户构造并提供*/    
    LidarDrv.clearRxBuffer = Clear_rx;
    LidarDrv.clearTxBuffer = Clear_tx;
    LidarDrv.sendData = Send_chars;
    LidarDrv.waitData = Wait_chars;
    LidarDrv.recvData = Recv_chars;
    LidarDrv.createTask = CreateTask;
    LidarDrv.deleteTask = DeleteTask;
}

/*雷达TCP相关函数初始化*/
void LidarTcpFuncInit (void)
{
    /*以下函数需要用户构造并提供*/    
    LidarDrv.clearRxBuffer = TCP_Clear_rx;
    LidarDrv.clearTxBuffer = TCP_Clear_tx;
    LidarDrv.sendData = TCP_Send_chars;
    LidarDrv.waitData = TCP_Wait_chars;
    LidarDrv.recvData = TCP_Recv_chars;
    LidarDrv.createTask = CreateTask;
    LidarDrv.deleteTask = DeleteTask;
}


/*向雷达发送命令*/
static bool sendLidarCommand (unsigned char cmd, const void *payload, unsigned char payloadsize)
{	
    unsigned char pkt_header[2] = {0};
    rplidar_cmd_header_t *header = (rplidar_cmd_header_t *)pkt_header;

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmdByte = cmd;

    if (LidarDrv.sendData((char*)pkt_header, 2) != 2)
        return FALSE;

    if ((payloadsize != 0) && (payload != NULL))
    {
        unsigned char checksum = 0;
		
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= payloadsize;

        for (unsigned char pos = 0; pos < payloadsize; ++pos) 
        {
            checksum ^= ((char*)payload)[pos];
        }
		
        if (LidarDrv.sendData((char*)&payloadsize, 1) != 1)
            return FALSE;

        if (LidarDrv.sendData((char*)payload, payloadsize) != payloadsize)
            return FALSE;
	
        if (LidarDrv.sendData((char*)&checksum, 1) != 1)
            return FALSE;			
    }

    return TRUE;
}

/*获取雷达设备信息*/
static bool _getDeviceInfo (unsigned int time_out_ms)
{		    
    if (LidarDrv.currentStatus == IS_SCANNING)
        LidarDrv.stopScan();
    
    int wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	

    rplidar_ans_header_t response_header;

    unsigned int  size_a = sizeof(rplidar_ans_header_t);
    unsigned int  size_b = sizeof(rplidar_response_device_info_t);
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();	
	
    if (sendLidarCommand (RPLIDAR_CMD_GET_DEVICE_INFO, NULL, 0) == TRUE)
    {	
        wait_size = size_a;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = size_b;
        
        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
            {
                if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2) && (response_header.type == RPLIDAR_ANS_TYPE_DEVINFO))
                {
                    wait_size = size_b;    curr_wait_size = wait_size;    prev_wait_size = size_a;    next_wait_size = 0;
                
                    if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == wait_size)
                    {		
                        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                        {
                            if (LidarDrv.recvData ((char*)&(LidarDrv.lidar_info), wait_size, &data_buffer) == wait_size)
                            {
                                return TRUE;
                            }
                        }
                    }
                }
            }
        }
    }
    
    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;	
}

/*查询雷达健康信息*/
static bool _getHealth(unsigned int time_out_ms)
{		    
    if (LidarDrv.isConnected == FALSE)
        return FALSE; 

    if (LidarDrv.currentStatus == IS_SCANNING)
        LidarDrv.stopScan();
    
    int wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	

    rplidar_ans_header_t response_header;

    unsigned int  size_a = sizeof(rplidar_ans_header_t);
    unsigned int  size_b = sizeof(rplidar_response_device_health_t);
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();	
	
    if (sendLidarCommand (RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0) == TRUE)
    {	
        wait_size = size_a;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = size_b;

        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
            {
                if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2) && (response_header.type == RPLIDAR_ANS_TYPE_DEVHEALTH))
                {
                    wait_size = size_b;    curr_wait_size = wait_size;    prev_wait_size = size_a;    next_wait_size = 0;

                    if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == wait_size)
                    {		
                        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                        {
                            if (LidarDrv.recvData ((char*)&(LidarDrv.health_info), wait_size, &data_buffer) == wait_size)
                            {
                                return TRUE;
                            }
                        }			
                    }
                }
            }
        }
    }

    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;	
}

/*询问雷达转接板是否支持motor control*/
static bool _checkMotorCtrlSupport (unsigned int time_out_ms)
{    
    if (LidarDrv.isConnected == FALSE)
        return FALSE;    

    if (LidarDrv.currentStatus == IS_SCANNING)
        LidarDrv.stopScan();

    int wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	
    
    rplidar_payload_acc_board_flag_t flag;
    
    rplidar_ans_header_t response_header;
    
    rplidar_response_acc_board_flag_t acc_board_flag;
    
    flag.reserved = 0;

    unsigned int  size_a = sizeof(rplidar_ans_header_t);
    unsigned int  size_b = sizeof(rplidar_response_acc_board_flag_t);
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();    
    
    if (sendLidarCommand (RPLIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag)) == TRUE) 
    {
        wait_size = size_a;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = size_b;

        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
            {
                if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2) && (response_header.type == RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG))
                {
                    wait_size = size_b;    curr_wait_size = wait_size;    prev_wait_size = size_a;    next_wait_size = 0;  
                         
                    if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == wait_size)
                    {  
                        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                        {
                            if (LidarDrv.recvData ((char*)&acc_board_flag, wait_size, &data_buffer) == wait_size)
                            {
                                if (acc_board_flag.support_flag & RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK)
                                {
                                    LidarDrv.isSupportingMotorCtrl = TRUE;
                                }
                                else
                                {
                                    LidarDrv.isSupportingMotorCtrl = FALSE;
                                }
                                
                                return TRUE;
                            }
                        }
                    }
                } 
            }
        }
    }

    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;	
}
/*查询雷达采样频率信息*/
static bool getSampleDuration_uS (unsigned int time_out_ms)
{
    if (LidarDrv.isConnected == FALSE)
        return FALSE; 

    if (LidarDrv.currentStatus == IS_SCANNING)
        LidarDrv.stopScan();
    
    int wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	

    rplidar_ans_header_t response_header;

    unsigned int  size_a = sizeof(rplidar_ans_header_t);
    unsigned int  size_b = sizeof(rplidar_response_sample_rate_t);
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();	
	
    if (sendLidarCommand (RPLIDAR_CMD_GET_SAMPLERATE, NULL, 0) == TRUE)
    {	
        wait_size = size_a;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = size_b;

        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
            {
                if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2) && (response_header.type == RPLIDAR_ANS_TYPE_SAMPLE_RATE))
                {
                    wait_size = size_b;    curr_wait_size = wait_size;    prev_wait_size = size_a;    next_wait_size = 0;

                    if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == wait_size)
                    {		
                        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                        {
                            if (LidarDrv.recvData ((char*)&(LidarDrv.rate_info), wait_size, &data_buffer) == wait_size)
                            {	
                                return TRUE;
                            }
                        }			
                    }
                }
            }
        }
    }

    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;	
} 

/*检查雷达是否支持查询配置信息*/
static bool checkSupportConfigCommands (void)
{
    if ((LidarDrv.lidar_info.firmware_version_major >= 1) && (LidarDrv.lidar_info.firmware_version_minor >= 24))     // if lidar firmware >= 1.24
    {
        return TRUE;
    }

    return FALSE;
}

/*检查雷达是否支持ExpressScan*/
static bool checkExpressScanSupported (void)
{
    if ((LidarDrv.lidar_info.firmware_version_major >= 1) && (LidarDrv.lidar_info.firmware_version_minor >= 17))     // if lidar firmware >= 1.17
    {
        return TRUE;
    }

    return FALSE;

}

/*查询雷达配置信息*/
static bool getLidarConf (unsigned int cmd_type, char *reserved_addr, unsigned int reserved_size, char **ret_addr, unsigned int *ret_size, unsigned int time_out_ms)
{		    
    if (LidarDrv.isConnected == FALSE)
        return FALSE;
            
    if (reserved_size > (sizeof(rplidar_payload_get_scan_conf_t) - 4))
        return FALSE;

    if ((ret_addr == NULL) || (ret_size == NULL))    //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE;
        
    if (LidarDrv.currentStatus == IS_SCANNING)
        LidarDrv.stopScan();
        
    int wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	

    rplidar_payload_get_scan_conf_t query;

    rplidar_ans_header_t response_header;

    memset(&query, 0, sizeof(query));
    
    query.type = cmd_type;    
    
    if ((reserved_addr != NULL) && (reserved_size > 0))
    {
        for (int i = 0; i < reserved_size; i++)
        {
            query.reserved[i] = reserved_addr[i];
        }
    }

    unsigned int  size_a = sizeof(rplidar_ans_header_t);
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();	
	
    if (sendLidarCommand (RPLIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(rplidar_payload_get_scan_conf_t)) == TRUE)
    {	
        wait_size = size_a;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

        if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
            {
                if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2) && (response_header.type == RPLIDAR_ANS_TYPE_GET_LIDAR_CONF))
                {
                    wait_size = response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK;    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;
            
                    if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                    {
                        unsigned int reply_type = -1;
                        unsigned int type_len = sizeof(cmd_type);
                      
                        if (LidarDrv.recvData ((char*)&reply_type, type_len, &data_buffer) == type_len)
                        {
                            if (reply_type == cmd_type)
                            {
                                char *temp_buffer = (char*)malloc (wait_size - type_len);    //malloc申请的内存必须用free释放；
                                
                                memset((char*)temp_buffer, 0, (wait_size - type_len));
                                
                                if (LidarDrv.recvData (temp_buffer, (wait_size - type_len), &data_buffer) == (wait_size - type_len))
                                {	
                                    *ret_addr = temp_buffer;
                                    *ret_size = wait_size - type_len;
                                    
                                    return TRUE;
                                }
                            }
                        }
                    }			
                }
            }
        }
    }

    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;	
}

/*获取支持的扫描模式数量*/
static bool getScanModeCount (unsigned int time_out_ms)
{
    char *data_buffer = NULL;
    unsigned int data_size = 0;
    unsigned int temp_data = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_COUNT, NULL, 0, &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size <= sizeof(LidarDrv.ScanModeCount))    //注意：长度检查，以保证内存操作不越界！！！
        {
            //memcpy ((char*)&temp_data, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                ((char*)&temp_data)[i] = data_buffer[i];;
            }
            LidarDrv.ScanModeCount = temp_data;
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取雷达在给定扫描工作模式的采样点时间间隔，单位微秒，1秒(1000000微秒)除以这个时间间隔即可得到采样频率*/
static bool getLidarSampleDuration (float *sample_duration_res, unsigned short scan_mode_id, unsigned int time_out_ms)
{
    if (sample_duration_res == NULL) //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE; 

    char *data_buffer = NULL;
    unsigned int data_size = 0;
    unsigned long long temp_data = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, (char*)&scan_mode_id, sizeof(scan_mode_id), &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size <= sizeof(*sample_duration_res))    //注意：长度检查，以保证内存操作不越界！！！
        {
            //memcpy ((char*)&temp_data, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                ((char*)&temp_data)[i] = data_buffer[i];
            }
            *sample_duration_res = (float)(temp_data >> 8);
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取雷达在给定扫描工作模式下最大采样距离*/
static bool getMaxDistance (float *max_mistance, unsigned short scan_mode_id, unsigned int time_out_ms)
{
    if (max_mistance == NULL)        //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE; 

    char *data_buffer = NULL;
    unsigned int data_size = 0;
    unsigned long long temp_data = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, (char*)&scan_mode_id, sizeof(scan_mode_id), &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size <= sizeof(*max_mistance))    //注意：长度检查，以保证内存操作不越界！！！
        {
            //memcpy ((char*)&temp_data, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                ((char*)&temp_data)[i] = data_buffer[i];
            }
            *max_mistance = (float)(temp_data >> 8);
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取雷达在给定扫描工作模式下EXPRESS_SCAN命令请求采用的协议版本*/
static bool getScanModeAnsType (unsigned char *ans_type, unsigned short scan_mode_id, unsigned int time_out_ms)
{
    if (ans_type == NULL)            //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE;    
    
    char *data_buffer = NULL;
    unsigned int data_size = 0;
    unsigned int temp_data = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, (char*)&scan_mode_id, sizeof(scan_mode_id), &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size <= sizeof(*ans_type))    //注意：长度检查，以保证内存操作不越界！！！
        {
            //memcpy ((char*)&temp_data, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                ((char*)&temp_data)[i] = data_buffer[i];;
            }
            *ans_type = (unsigned char)temp_data;
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取雷达在给定扫描工作模式下所对应的可供阅读的名称*/
static bool getScanModeName (char *mode_name, unsigned short scan_mode_id, unsigned int time_out_ms)
{
    if (mode_name == NULL)           //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE;
    
    char *data_buffer = NULL;
    unsigned int data_size = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_NAME, (char*)&scan_mode_id, sizeof(scan_mode_id), &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size < 64)                                               //注意：传入的的指针指向的内存区域至少应有64字节，且最多只应接受63个返回的字符，最后一个字符元素必须是0，字符数组才能成为一个字符串数组；
        {
            //memcpy ((char*)mode_name, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                mode_name[i] = data_buffer[i];
            }
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取雷达推荐的扫描工作模式的ID*/
static bool getTypicalScanMode (unsigned short *typical_mode_id, unsigned int time_out_ms)
{
    if (typical_mode_id == NULL)     //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE;
    
    char *data_buffer = NULL;
    unsigned int data_size = 0;

    if (getLidarConf (RPLIDAR_CONF_SCAN_MODE_TYPICAL, NULL, 0, &data_buffer, &data_size, time_out_ms) == TRUE)
    {
        if (data_size <= sizeof(*typical_mode_id))    //注意：长度检查，以保证内存操作不越界！！！                                          
        {
            //memcpy ((char*)typical_mode_id, &data_buffer[0], data_size);        //这条语句的执行效果和下面的for循环一样，用哪一个都可以；
            for (int i = 0; i < data_size; i++)
            {
                ((char*)typical_mode_id)[i] = data_buffer[i];
            }
            
            free (data_buffer);        //在getLidarConf()中malloc申请的内存必须用free释放；
            
            return TRUE;
        }
        
        free (data_buffer);            //在getLidarConf()中malloc申请的内存必须用free释放；
    }
    
    return FALSE;
}

/*获取所有支持的扫描模式的详细信息
static bool _getAllSupportedScanModes (RplidarScanMode **scan_mode_array, unsigned short *scan_mode_count, unsigned int time_out_ms)
{
    if ((scan_mode_count == NULL) || (scan_mode_array == NULL))    //注意：传入的指针不能为空，避免非法操作！！！
        return FALSE;
    
    if (checkSupportConfigCommands() == TRUE)    
    {
        if (getScanModeCount(time_out_ms) == TRUE)
        {
            if (LidarDrv.ScanModeCount > 0)
            {                
                RplidarScanMode *scan_mode_temp = (RplidarScanMode*)malloc (LidarDrv.ScanModeCount * sizeof(RplidarScanMode));     //malloc申请的内存必须用free释放；
                
                memset((char*)scan_mode_temp, 0, (LidarDrv.ScanModeCount * sizeof(RplidarScanMode)));
                
                for (unsigned short i = 0; i < LidarDrv.ScanModeCount; i++)
                {
                    scan_mode_temp[i].id = i;
                    
                    if (getLidarSampleDuration ((float*)&(scan_mode_temp[i].us_per_sample), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getMaxDistance ((float*)&(scan_mode_temp[i].max_distance), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getScanModeAnsType(&(scan_mode_temp[i].ans_type), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getScanModeName (scan_mode_temp[i].scan_mode, i, 1000) == FALSE)
                        return FALSE;
                }
                
                unsigned short temp;
                
                if (LidarDrv.ScanModeCount > MAX_SCAN_MODE_COUNT)
                    temp = MAX_SCAN_MODE_COUNT;
                else
                    temp = LidarDrv.ScanModeCount;
                
                for (unsigned short j = 0; j < temp; j++)    //将获取到的雷达扫描模式的详细信息填充到雷达驱动结构体，最多可填充MAX_SCAN_MODE_COUNT个，剩下的无法填充进去；
                {
                    LidarDrv.ScanMode[j].id = j;
                    
                    LidarDrv.ScanMode[j].us_per_sample = scan_mode_temp[j].us_per_sample;
                    
                    LidarDrv.ScanMode[j].max_distance = scan_mode_temp[j].max_distance;
                    
                    LidarDrv.ScanMode[j].ans_type = scan_mode_temp[j].ans_type;
                    
                    memcpy ((char*)(LidarDrv.ScanMode[j].scan_mode), (char*)(scan_mode_temp[j].scan_mode), 63);
                }
                
                getTypicalScanMode (&(LidarDrv.TypicalScanModeID), 1000);

                *scan_mode_count = LidarDrv.ScanModeCount;
                *scan_mode_array = scan_mode_temp;
                
                return TRUE;
            }
        }
    }
    else if (checkExpressScanSupported() == TRUE)
    {
        if (getSampleDuration_uS(time_out_ms) == TRUE)
        {
            LidarDrv.ScanModeCount = 2;
            
            RplidarScanMode *scan_mode_temp = (RplidarScanMode*)malloc (LidarDrv.ScanModeCount * sizeof(RplidarScanMode));     //malloc申请的内存必须用free释放；
                
            memset((char*)scan_mode_temp, 0, (LidarDrv.ScanModeCount * sizeof(RplidarScanMode)));
            
            scan_mode_temp[0].id = RPLIDAR_CONF_SCAN_COMMAND_STD;
            scan_mode_temp[0].us_per_sample = 500.0;
            scan_mode_temp[0].max_distance = 12.0;
            scan_mode_temp[0].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
            strcpy (scan_mode_temp[0].scan_mode, "Standard");
            
            scan_mode_temp[1].id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            scan_mode_temp[1].us_per_sample = 250.0;
            scan_mode_temp[1].max_distance = 12.0;
            scan_mode_temp[1].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy (scan_mode_temp[1].scan_mode, "Express"); 
            
            LidarDrv.ScanMode[0].id = RPLIDAR_CONF_SCAN_COMMAND_STD;
            LidarDrv.ScanMode[0].us_per_sample = 500.0;
            LidarDrv.ScanMode[0].max_distance = 12.0;
            LidarDrv.ScanMode[0].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
            strcpy (LidarDrv.ScanMode[0].scan_mode, "Standard");
            
            LidarDrv.ScanMode[1].id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            LidarDrv.ScanMode[1].us_per_sample = 250.0;
            LidarDrv.ScanMode[1].max_distance = 12.0;
            LidarDrv.ScanMode[1].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy (LidarDrv.ScanMode[1].scan_mode, "Express"); 
            
            *scan_mode_count = LidarDrv.ScanModeCount;
            *scan_mode_array = scan_mode_temp;
            
            return TRUE;           
        }
    }

    return FALSE;
}*/

/*获取所有支持的扫描模式的详细信息*/
static bool _getAllSupportedScanModes (unsigned int time_out_ms)
{
    if (checkSupportConfigCommands() == TRUE)    
    {
        if (getScanModeCount(time_out_ms) == TRUE)
        {
            if (LidarDrv.ScanModeCount > 0)
            {   
                unsigned short temp;
                
                if (LidarDrv.ScanModeCount > MAX_SCAN_MODE_COUNT)
                    temp = MAX_SCAN_MODE_COUNT;
                else
                    temp = LidarDrv.ScanModeCount;
                
                for (unsigned short i = 0; i < temp; i++)
                {
                    LidarDrv.ScanMode[i].id = i;
                    
                    if (getLidarSampleDuration ((float*)(&(LidarDrv.ScanMode[i].us_per_sample)), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getMaxDistance ((float*)(&(LidarDrv.ScanMode[i].max_distance)), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getScanModeAnsType(&(LidarDrv.ScanMode[i].ans_type), i, 1000) == FALSE)
                        return FALSE;
                    
                    if (getScanModeName (LidarDrv.ScanMode[i].scan_mode, i, 1000) == FALSE)
                        return FALSE;
                }
                
                getTypicalScanMode (&(LidarDrv.TypicalScanModeID), 1000);
                
                return TRUE;
            }
        }
    }
    else if (checkExpressScanSupported() == TRUE)
    {
        if (getSampleDuration_uS(time_out_ms) == TRUE)
        {
            LidarDrv.ScanModeCount = 2;
            
            LidarDrv.ScanMode[0].id = RPLIDAR_CONF_SCAN_COMMAND_STD;
            LidarDrv.ScanMode[0].us_per_sample = 500.0;
            LidarDrv.ScanMode[0].max_distance = 12.0;
            LidarDrv.ScanMode[0].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
            strcpy (LidarDrv.ScanMode[0].scan_mode, "Standard");
            
            LidarDrv.ScanMode[1].id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            LidarDrv.ScanMode[1].us_per_sample = 250.0;
            LidarDrv.ScanMode[1].max_distance = 12.0;
            LidarDrv.ScanMode[1].ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy (LidarDrv.ScanMode[1].scan_mode, "Express"); 
            
            return TRUE;           
        }
    }

    return FALSE;
}

/*检查雷达连接状态*/
static bool checkLidarConnection (unsigned int time_out_ms)
{
    if (LidarDrv.isConnected == FALSE)
        return FALSE;

    rplidar_response_device_info_t temp_lidar_info;
    
    memset ((char*)&(temp_lidar_info), 0, sizeof(rplidar_response_device_info_t));
    
    temp_lidar_info = LidarDrv.lidar_info;
    
    if (_getDeviceInfo(time_out_ms) == TRUE)
    {
        for (unsigned short i = 0; i < 16; i++)
        {
            if (temp_lidar_info.serialnum[i] != LidarDrv.lidar_info.serialnum[i])        
            {
                LidarDrv.lidar_info = temp_lidar_info;
                return FALSE;
            }  
        }          
    }

    return TRUE;        
}

/*识别传入的扫描模式*/
static unsigned short identifyScanMode (unsigned short scan_mode_id)
{
    if (scan_mode_id >= MAX_SCAN_MODE_COUNT)
        return 0xFFFF;
    
    if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "Standard") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_STD;
    }
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "Express") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
    }
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "HQ") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_HQ;
    }    
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "Boost") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_BOOST;
    }    
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "Stability") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_STABILITY;
    }    
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "Sensitivity") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY;
    }
    else if (strcmp((char*)(LidarDrv.ScanMode[scan_mode_id].scan_mode), "DenseBoost") == 0)
    {
        return RPLIDAR_CONF_SCAN_COMMAND_DENSEBOOST;
    }          
    else
    {
        return 0xFFFF;
    }           
}

/*开始扫描*/
static bool _startScan (bool force_scan_flag, unsigned short scan_mode_id, unsigned int time_out_ms)
{
    if (checkLidarConnection(1000) == FALSE)    //如果雷达连接以丢失或者不匹配，应当返回FALSE；
        return FALSE;

    if (_getHealth(1000) == FALSE)              //如果查询不到雷达健康信息，应当返回FALSE；
        return FALSE;

    if (LidarDrv.health_info.status == 0x02)    //如果雷达内部有故障，应当返回FALSE；
        return FALSE;

    if (LidarDrv.currentStatus == IS_SCANNING)  //如果当前已处于扫描状态，需先停止扫描；
        LidarDrv.stopScan();
    
    _startMotor ();
                
    int  wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;		
    unsigned char temp_scan_cmd = RPLIDAR_CMD_SCAN;
    
    rplidar_payload_express_scan_t scan_payload;
    rplidar_ans_header_t response_header;
        
    memset(&scan_payload, 0, sizeof(scan_payload));
    
    LidarDrv.clearRxBuffer();
    LidarDrv.clearTxBuffer();
	
	if (force_scan_flag == TRUE)
	{
	    scan_mode_id = 0;
	    
	    temp_scan_cmd = RPLIDAR_CMD_FORCE_SCAN;
	}
	
	LidarDrv.current_RPM = DEFAULT_MOTOR_RPM;
	
	switch (identifyScanMode(scan_mode_id))
	{
	    case RPLIDAR_CONF_SCAN_COMMAND_STD :
	    	        
	        if (sendLidarCommand (temp_scan_cmd, NULL, 0) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_measurement_node_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT)
                                { 
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_STD;
                                    
                                    if (LidarDrv.createTask(cacheScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            } 
                        }
                    }
                }
            }
	        break;
	        
	    case RPLIDAR_CONF_SCAN_COMMAND_EXPRESS :
	        	        
	        scan_payload.working_mode = 0x00;    //Express模式，固定填0x00；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
                                    
                                    if (LidarDrv.createTask(cacheCapsuledScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheCapsuledScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }	    
	        break;
	        
	    case RPLIDAR_CONF_SCAN_COMMAND_HQ :
	        	        
	        scan_payload.working_mode = scan_mode_id;    //HQ模式，填scan_mode_id；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_HQ)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_HQ;
                                    
                                    if (LidarDrv.createTask(cacheHqScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheHqScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }	    
	        break;	 
	               
	    case RPLIDAR_CONF_SCAN_COMMAND_BOOST :
	        	        
	        scan_payload.working_mode = scan_mode_id;    //Boost模式，填scan_mode_id；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_BOOST;
                                    
                                    if (LidarDrv.createTask(cacheUltraCapsuledScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheUltraCapsuledScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }		    
	        break;	        	        	        

	    case RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY :
	        	        
	        scan_payload.working_mode = scan_mode_id;    //Sensitivity模式，填scan_mode_id；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_SENSITIVITY;
                                    
                                    if (LidarDrv.createTask(cacheUltraCapsuledScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheUltraCapsuledScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }		    
	        break;
	        
	    case RPLIDAR_CONF_SCAN_COMMAND_STABILITY :
	        	        
	        scan_payload.working_mode = scan_mode_id;    //Stability模式，填scan_mode_id；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_STABILITY;
                                    
                                    if (LidarDrv.createTask(cacheUltraCapsuledScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheUltraCapsuledScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }		    
	        break;

	    case RPLIDAR_CONF_SCAN_COMMAND_DENSEBOOST :
	        	        
	        scan_payload.working_mode = 0x00;    //和Express模式一样，固定填0x00；
	        
	        if (sendLidarCommand (RPLIDAR_CMD_EXPRESS_SCAN, &scan_payload, sizeof(rplidar_payload_express_scan_t)) == TRUE)
            {	
                wait_size = sizeof(rplidar_ans_header_t);    curr_wait_size = wait_size;    prev_wait_size = 0;    next_wait_size = 0;

                if (LidarDrv.waitData (wait_size, &time_out_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&response_header, wait_size, &data_buffer) == wait_size)
                    {
                        if ((response_header.syncByte1 == RPLIDAR_ANS_SYNC_BYTE1) && (response_header.syncByte2 == RPLIDAR_ANS_SYNC_BYTE2))
                        {
                            if ((response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK) == sizeof(rplidar_response_dense_capsule_measurement_nodes_t)) 
                            {
                                if (response_header.type == RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED)
                                {
                                    LidarDrv.currentScanModeID = scan_mode_id;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_DENSEBOOST;
                                    
                                    if (LidarDrv.createTask(cacheCapsuledScanData, NULL) == TRUE)
                                    {
                                        LidarDrv.current_cache_proc = cacheCapsuledScanData;
                                        return TRUE;
                                    }
                                    
                                    LidarDrv.currentScanModeID = RPLIDAR_CONF_SCAN_MODE_ID_NOT_VALID;
                                    LidarDrv.currentScanMode = RPLIDAR_CONF_SCAN_COMMAND_NOT_VALID;
                                }
                            }    
                        }
                    }
                }
            }		    
	        break;
	        	        	
	    default :	    
	        break;	                	        	
	}

    _stopMotor ();

    curr_wait_size = 0;    prev_wait_size = 0;    next_wait_size = 0;
    
    return FALSE;
}

/*停止扫描*/
static bool _stopScan ()
{
    _stopMotor ();
    
    HAL_Delay (10);
    
    sendLidarCommand (RPLIDAR_CMD_STOP, NULL, 0);

    if (LidarDrv.deleteTask(LidarDrv.current_cache_proc) == TRUE)
    {
        return TRUE;
    }
    
    return FALSE;
}

/*重置雷达*/
static bool _resetLidar (void)                                 
{
    if (_stopScan () == TRUE)
    {
        LidarDrv.currentStatus = IS_STOPPED;
    }
        
    HAL_Delay (2000);

    if (sendLidarCommand (RPLIDAR_CMD_RESET, NULL, 0) == TRUE)
    {
        LidarDrv.currentStatus = IS_RESET;

        return TRUE;
    }
    
    return FALSE;
}


/*启动电机*/
static bool _startMotor (void)
{
    if (LidarDrv.isSupportingMotorCtrl == TRUE)
    {
        LidarDrv.current_PWM = DEFAULT_MOTOR_PWM;
        
        return _setMotorPWM (DEFAULT_MOTOR_PWM);
    }
    else
    {
        return Set_dtr_signal(TRUE);                     //置A1转接板DTR为TRUE，令A1旋转;
    }
}

/*停止电机*/
static bool _stopMotor (void)
{
    if (LidarDrv.isSupportingMotorCtrl == TRUE)
    {
        return _setMotorPWM (0);
    }
    else
    {        
        return Set_dtr_signal(FALSE);                    //置A1转接板DTR为FALSE，令A1停止旋转;
    }
}


/*设定外部PWM*/
static bool _setMotorPWM (unsigned short pwm)
{
    rplidar_payload_motor_pwm_t motor_pwm;
    
    if (pwm > MAX_MOTOR_PWM)
        pwm = MAX_MOTOR_PWM;
    
    motor_pwm.pwm_value = pwm;

    MX_TIM3_Set_Pwm (pwm);
    
    return TRUE;
}

/*请求设置RPLIDAR测距核心的旋转速度(Rpm)*/
static bool _setMotorRpm (unsigned short rpm)
{
    rplidar_payload_motor_pwm_t motor_rpm;
    
    if (rpm > MAX_MOTOR_RPM)
        rpm = MAX_MOTOR_RPM;
        
    motor_rpm.pwm_value = rpm;

    return sendLidarCommand (RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL, &motor_rpm, sizeof(motor_rpm));
}

/*获取并解析ScanData*/
static void cacheScanData(void* arg)
{
    LidarDrv.currentStatus = IS_SCANNING;

    int hq_node_count = 0;

    int temp_node_count = 0;
    
    int temp_count = 0;

    bool first_circle_come = FALSE;
    
    unsigned char temp_byte[2] = {0};
    
    int  wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	

    unsigned int  size_a = 0;
    unsigned int  size_b = 0;
    unsigned int  size_c = 0;
    
    #ifdef _CALC_DRIVER_PROCESS_TIME_
    static  bool  one_frame_come_flag = FALSE;
    unsigned int  one_frame_start_time_ms = 0;
    unsigned int  one_frame_finish_time_ms = 0;
    unsigned int  one_frame_process_time_ms = 0;
    unsigned int  frame_interval_ms = 0;
    unsigned int  wait_cost_time_ms = 0;
    #endif
        
    unsigned int  wait_time_ms = 0;
    unsigned int  max_process_time_ms = 0;
   
    static rplidar_response_measurement_node_t  temp_node_buf[32];

    size_a = sizeof(temp_node_buf[0].sync_quality);
    size_b = sizeof(temp_node_buf[0].angle_q6_checkbit);
    size_c = sizeof(temp_node_buf) - sizeof(temp_node_buf[0].sync_quality) - sizeof(temp_node_buf[0].angle_q6_checkbit); 

    wait_time_ms = 500;

    wait_size = sizeof(temp_node_buf);
    
    if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
    {
        LidarDrv.recvData ((char*)temp_node_buf, wait_size, &data_buffer);
    }
       
    while(LidarDrv.currentStatus == IS_SCANNING)
    {
        wait_time_ms = 500;
      
        wait_size = size_a;
        
        #ifdef _CALC_DRIVER_PROCESS_TIME_
        if (one_frame_come_flag == FALSE)
        {
            one_frame_start_time_ms = getms ();    //获取系统当前时间；
            
            one_frame_come_flag = TRUE;
        }
        #endif
        
        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)temp_byte, wait_size, &data_buffer) != wait_size)
                continue;
            
            temp_byte[1] = temp_byte[0];
                       
            if ((!(temp_byte[0] & 0x01)) == ((temp_byte[1] & 0x02) >> 1))
            {
                temp_node_buf[0].sync_quality = temp_byte[0];
                    
                wait_size = size_b;
                
                if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)temp_byte, wait_size, &data_buffer) != wait_size)
                        continue;
                    
                    if ((temp_byte[0] & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == 1)
                    {
                        temp_node_buf[0].angle_q6_checkbit = *((unsigned short*)temp_byte);
                            
                        wait_size = size_c;
                
                        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                        {
                            #ifdef _CALC_DRIVER_PROCESS_TIME_
                            if (wait_time_ms < 500)
                            {
                                wait_cost_time_ms += 500 - wait_time_ms;
                            }
                            #endif
                          
                            if (LidarDrv.recvData ((char*)&(temp_node_buf[0].distance_q2), wait_size, &data_buffer) != wait_size)
                                continue;
                            
                            temp_node_count = sizeof(temp_node_buf) / sizeof(temp_node_buf[0]);
                            
                            temp_count = temp_node_count;
                            
                            while(temp_node_count > 0)    
                            {
                                if (((LidarDrv.bufferState_A == IS_EMPTY) || (LidarDrv.bufferState_A == IS_WRITING)) && (LidarDrv.bufferState_B != IS_WRITING) && (temp_node_count > 0))
                                {
                                    LidarDrv.bufferState_A = IS_WRITING;
                                  
                                    unsigned int j = 0;
                                    
                                    if (temp_node_count < temp_count)
                                    {
                                        j = temp_count - temp_node_count;
                                    }
                                    
                                    for ( ; j < temp_count; j++)
                                    {
                                        if ( (!(temp_node_buf[j].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == ((temp_node_buf[j].sync_quality & (RPLIDAR_RESP_MEASUREMENT_SYNCBIT << 1)) >> 1)) 
                                             && 
                                             ((temp_node_buf[j].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == 1) )
                                        {
                                            LidarDrv.nodeBuffer_A[hq_node_count].angle_z_q14 = (((temp_node_buf[j].angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;
                                            
                                            LidarDrv.nodeBuffer_A[hq_node_count].dist_mm_q2 = temp_node_buf[j].distance_q2;
                                            
                                            LidarDrv.nodeBuffer_A[hq_node_count].flag = temp_node_buf[j].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
                                            
                                            LidarDrv.nodeBuffer_A[hq_node_count].quality = (temp_node_buf[j].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << 
                                                                                                                                   RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                                            
                                            hq_node_count++;
                                        
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_node_buf[j].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) //当temp_node_buf[j].flag第一次为1的时候，表明已经收到了第一圈数据；                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                                
                                                    hq_node_count = 0;                                  //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                              
                                                    continue;
                                                }
                                            
                                                LidarDrv.validNodePos_A = hq_node_count;
                                            
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_B != IS_READING)    //如果buffer_B没有被读取，那么应当将buffer_B置为IS_EMPTY，后续数据应当向buffer_B中存储；   
                                                {
                                                    LidarDrv.bufferState_B = IS_EMPTY;
                                           
                                                    LidarDrv.bufferState_A = IS_READY;
                                            
                                                    LidarDrv.ready_time_ms_A = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_A));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_A * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                    
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);

                                                        wait_cost_time_ms = 0;

                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                        
                                                    j++;         //j++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_B正在被读取，那么后续数据应当继续向当前的buffer_A中存储；
                                                }
                                            }
                                        }
                                        else
                                        {
                                            temp_count = j;
                                            
                                            break;
                                        }
                                    }
                                        
                                    temp_node_count = temp_count - j;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则j==temp_count，那么此处就是0；
                                }
                                
                                if (((LidarDrv.bufferState_B == IS_EMPTY) || (LidarDrv.bufferState_B == IS_WRITING)) && (LidarDrv.bufferState_A != IS_WRITING) && (temp_node_count > 0))
                                {
                                    LidarDrv.bufferState_B = IS_WRITING;
                                    
                                    unsigned int k = 0;
                                    
                                    if (temp_node_count < temp_count)
                                    {
                                        k = temp_count - temp_node_count;
                                    }
                                    
                                    for ( ; k < temp_count; k++)
                                    {
                                        if ( (!(temp_node_buf[k].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == ((temp_node_buf[k].sync_quality & (RPLIDAR_RESP_MEASUREMENT_SYNCBIT << 1)) >> 1))  
                                             && 
                                             ((temp_node_buf[k].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == 1) )
                                        {
                                            LidarDrv.nodeBuffer_B[hq_node_count].angle_z_q14 = (((temp_node_buf[k].angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;
                                            
                                            LidarDrv.nodeBuffer_B[hq_node_count].dist_mm_q2 = temp_node_buf[k].distance_q2;
                                            
                                            LidarDrv.nodeBuffer_B[hq_node_count].flag = temp_node_buf[k].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
                                        
                                            LidarDrv.nodeBuffer_B[hq_node_count].quality = (temp_node_buf[k].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << 
                                                                                                                                       RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                                        
                                            hq_node_count++;
                                         
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_node_buf[k].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                    
                                                    hq_node_count = 0;                       //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                                
                                                    continue;
                                                }
                                           
                                                LidarDrv.validNodePos_B = hq_node_count;
                                         
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_A != IS_READING)    //如果buffer_A没有被读取，那么应当将buffer_A置为IS_EMPTY，后续数据应当向buffer_A中存储；   
                                                {
                                                    LidarDrv.bufferState_A = IS_EMPTY;
                                          
                                                    LidarDrv.bufferState_B = IS_READY;

                                                    LidarDrv.ready_time_ms_B = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_B));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_B * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                    
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);

                                                        wait_cost_time_ms = 0;

                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                    
                                                    k++;          //k++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_A正在被读取，那么后续数据应当继续向当前的buffer_B中存储；
                                                }
                                            }
                                        }    
                                        else
                                        {
                                            temp_count = k;
                                            
                                            break;
                                        }
                                    }
                                        
                                    temp_node_count = temp_count - k;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则k==temp_count，那么此处就是0；
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    LidarDrv.currentStatus = IS_STOPPED;
     
    return;
}

/*获取并解析HqScanData*/
static void cacheHqScanData(void* arg)
{
    LidarDrv.currentStatus = IS_SCANNING;
    
    while(LidarDrv.currentStatus == IS_SCANNING)
    {
      ;
    }
    
    return;
}

/*获取并解析CapsuledScanData*/
static void cacheCapsuledScanData(void* arg)
{
    LidarDrv.currentStatus = IS_SCANNING;

    int hq_node_count = 0;

    int temp_node_count = 0;
    
    int temp_count = 0;

    bool first_circle_come = FALSE;

    bool is_previous_capsuledataRdy = FALSE;    
   
    static rplidar_response_capsule_measurement_nodes_t  previous_capsule_node;
    
    static rplidar_response_measurement_node_hq_t temp_hq_buf[128];
    
    unsigned char temp_byte = 0;    //若定义为char，temp_byte是有符号数，作为判断表达式的左值的时，最高为是符号位，例如0xA5(1010 0101)，CPU会认为其是(-010 0101)即-0x35；
    
    int  wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	
    
    unsigned char check_sum = 0;
    unsigned char recv_check_sum = 0;
    
    unsigned int  size_a = 0;
    unsigned int  size_b = 0;
    unsigned int  size_c = 0;
    
    #ifdef _CALC_DRIVER_PROCESS_TIME_
    static  bool  one_frame_come_flag = FALSE;
    unsigned int  one_frame_start_time_ms = 0;
    unsigned int  one_frame_finish_time_ms = 0;
    unsigned int  one_frame_process_time_ms = 0;
    unsigned int  frame_interval_ms = 0;
    unsigned int  wait_cost_time_ms = 0;
    #endif
    
    unsigned int  wait_time_ms = 0;
    unsigned int  max_process_time_ms = 0;
    
    static rplidar_response_capsule_measurement_nodes_t  temp_capsule_node;

    size_a = sizeof(temp_capsule_node.s_checksum_1);
    size_b = sizeof(temp_capsule_node.s_checksum_2);
    size_c = sizeof(temp_capsule_node) - sizeof(temp_capsule_node.s_checksum_1) - sizeof(temp_capsule_node.s_checksum_2);
    
    while (LidarDrv.currentStatus == IS_SCANNING)
    {
        wait_time_ms = 500;
       
        wait_size = size_a;
        
        #ifdef _CALC_DRIVER_PROCESS_TIME_
        if (one_frame_come_flag == FALSE)
        {
            one_frame_start_time_ms = getms ();    //获取系统当前时间；
            
            one_frame_come_flag = TRUE;
        }
        #endif
        
        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&(temp_capsule_node.s_checksum_1), wait_size, &data_buffer) != wait_size)
                continue;
            
            temp_byte = temp_capsule_node.s_checksum_1;
            
            if ((temp_byte >> 4) == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1)
            {
                wait_size = size_b;
                
                if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&(temp_capsule_node.s_checksum_2), wait_size, &data_buffer) != wait_size)
                        continue;
                    
                    temp_byte = temp_capsule_node.s_checksum_2;
                    
                    if ((temp_byte >> 4) == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2)
                    {
                        wait_size = size_c;
                        
                        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                        {
                            #ifdef _CALC_DRIVER_PROCESS_TIME_
                            if (wait_time_ms < 500)
                            {
                                wait_cost_time_ms += 500 - wait_time_ms;
                            }
                            #endif
                          
                            if (LidarDrv.recvData ((char*)&(temp_capsule_node.start_angle_sync_q6), wait_size, &data_buffer) != wait_size)
                                continue;
                            
                            recv_check_sum = ((temp_capsule_node.s_checksum_1 & 0xF) | (temp_capsule_node.s_checksum_2 << 4));
                            
                            check_sum = 0;    //check_sum参与异或运算前先恢复至0，避免以前残留的值参与运算；
                            
                            for (unsigned int i = 0; i < wait_size; i++)
                            {
                                check_sum ^= ((unsigned char*)&(temp_capsule_node.start_angle_sync_q6))[i];
                            }
                            
                            if (check_sum == recv_check_sum)
                            {
                                if ((temp_capsule_node.start_angle_sync_q6) & (RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT))         //若该位为1，表明是一次全新的扫描过程（注意：不是新的一圈）；
                                {
                                    is_previous_capsuledataRdy = FALSE;
        
                                    hq_node_count = 0;
                                }
                                
                                if (is_previous_capsuledataRdy == TRUE)
                                {
                                    if (LidarDrv.currentScanMode == RPLIDAR_CONF_SCAN_COMMAND_EXPRESS)
                                    {
                                        capsuleToNormal (&previous_capsule_node, &temp_capsule_node, temp_hq_buf, &temp_node_count);
                                    }
                                    else if (LidarDrv.currentScanMode == RPLIDAR_CONF_SCAN_COMMAND_DENSEBOOST)
                                    {
                                        dense_capsuleToNormal (&previous_capsule_node, &temp_capsule_node, temp_hq_buf, &temp_node_count);
                                    }
                                }
                                
                                previous_capsule_node = temp_capsule_node;
                                    
                                is_previous_capsuledataRdy = TRUE;
                                
                                temp_count = temp_node_count;
                                
                                while(temp_node_count > 0)    
                                {
                                    if (((LidarDrv.bufferState_A == IS_EMPTY) || (LidarDrv.bufferState_A == IS_WRITING)) && (LidarDrv.bufferState_B != IS_WRITING) && (temp_node_count > 0))
                                    {
                                        LidarDrv.bufferState_A = IS_WRITING;
                                    
                                        unsigned int j = 0;
                                    
                                        if (temp_node_count < temp_count)
                                        {
                                            j = temp_count - temp_node_count;
                                        }
                                    
                                        for ( ; j < temp_count; j++)
                                        {
                                            LidarDrv.nodeBuffer_A[hq_node_count] = temp_hq_buf[j];
                                        
                                            hq_node_count++;
                                        
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_hq_buf[j].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) //当temp_hq_buf[j].flag第一次为1的时候，表明已经收到了第一圈数据（一般是不完整的）；                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                                
                                                    hq_node_count = 0;                                  //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                                
                                                    continue;
                                                }
                                            
                                                LidarDrv.validNodePos_A = hq_node_count;
                                            
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_B != IS_READING)    //如果buffer_B没有被读取，那么应当将buffer_B置为IS_EMPTY，后续数据应当向buffer_B中存储；   
                                                {
                                                    LidarDrv.bufferState_B = IS_EMPTY;
                                            
                                                    LidarDrv.bufferState_A = IS_READY;

                                                    LidarDrv.ready_time_ms_A = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_A));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_A * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                    
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);

                                                        wait_cost_time_ms = 0;

                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                        
                                                    j++;         //j++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_B正在被读取，那么后续数据应当继续向当前的buffer_A中存储；
                                                }
                                            }
                                        }
                                        
                                        temp_node_count = temp_count - j;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则j==temp_count，那么此处就是0；
                                    }
                                
                                    if (((LidarDrv.bufferState_B == IS_EMPTY) || (LidarDrv.bufferState_B == IS_WRITING)) && (LidarDrv.bufferState_A != IS_WRITING) && (temp_node_count > 0))
                                    {
                                        LidarDrv.bufferState_B = IS_WRITING;
                                    
                                        unsigned int k = 0;
                                    
                                        if (temp_node_count < temp_count)
                                        {
                                            k = temp_count - temp_node_count;
                                        }
                                    
                                        for ( ; k < temp_count; k++)
                                        {
                                            LidarDrv.nodeBuffer_B[hq_node_count] = temp_hq_buf[k]; 
                                        
                                            hq_node_count++;
                                        
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_hq_buf[k].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                    
                                                    hq_node_count = 0;                       //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                                
                                                    continue;
                                                }
                                            
                                                LidarDrv.validNodePos_B = hq_node_count;
                                            
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_A != IS_READING)    //如果buffer_A没有被读取，那么应当将buffer_A置为IS_EMPTY，后续数据应当向buffer_A中存储；   
                                                {
                                                    LidarDrv.bufferState_A = IS_EMPTY;
                                            
                                                    LidarDrv.bufferState_B = IS_READY;

                                                    LidarDrv.ready_time_ms_B = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_B));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_B * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                        
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);

                                                        wait_cost_time_ms = 0;
                                                        
                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                        
                                                    k++;          //k++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_A正在被读取，那么后续数据应当继续向当前的buffer_B中存储；
                                                }
                                            }
                                        }
                                        
                                        temp_node_count = temp_count - k;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则k==temp_count，那么此处就是0；
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    LidarDrv.currentStatus = IS_STOPPED;
    
    return;
}

/*获取并解析UltraCapsuledScanData*/
static void cacheUltraCapsuledScanData(void* arg)
{
    LidarDrv.currentStatus = IS_SCANNING;

    int hq_node_count = 0;

    int temp_node_count = 0;
    
    int temp_count = 0;

    bool first_circle_come = FALSE;

    bool is_previous_capsuledataRdy = FALSE;    
   
    static rplidar_response_ultra_capsule_measurement_nodes_t  previous_ultra_capsule_node;
    
    static rplidar_response_measurement_node_hq_t temp_hq_buf[128];
    
    unsigned char temp_byte = 0;    //若定义为char，temp_byte是有符号数，作为判断表达式的左值的时，最高为是符号位，例如0xA5(1010 0101)，CPU会认为其是(-010 0101)即-0x35；
    
    int  wait_size = 0;
    UART1_RX_BUFFER* data_buffer = NULL;	
    
    unsigned char check_sum = 0;
    unsigned char recv_check_sum = 0;
    
    unsigned int  size_a = 0;
    unsigned int  size_b = 0;
    unsigned int  size_c = 0;
    
    #ifdef _CALC_DRIVER_PROCESS_TIME_
    static  bool  one_frame_come_flag = FALSE;
    unsigned int  one_frame_start_time_ms = 0;
    unsigned int  one_frame_finish_time_ms = 0;
    unsigned int  one_frame_process_time_ms = 0;
    unsigned int  frame_interval_ms = 0;
    unsigned int  wait_cost_time_ms = 0;
    #endif
    
    unsigned int  wait_time_ms = 0;
    unsigned int  max_process_time_ms = 0;
    
    static rplidar_response_ultra_capsule_measurement_nodes_t  temp_ultra_capsule_node;

    size_a = sizeof(temp_ultra_capsule_node.s_checksum_1);
    size_b = sizeof(temp_ultra_capsule_node.s_checksum_2);
    size_c = sizeof(temp_ultra_capsule_node) - sizeof(temp_ultra_capsule_node.s_checksum_1) - sizeof(temp_ultra_capsule_node.s_checksum_2);
    
    while (LidarDrv.currentStatus == IS_SCANNING)
    {
        wait_time_ms = 500;
       
        wait_size = size_a;    
        
        #ifdef _CALC_DRIVER_PROCESS_TIME_
        if (one_frame_come_flag == FALSE)
        {
            one_frame_start_time_ms = getms ();    //获取系统当前时间；
            
            one_frame_come_flag = TRUE;
        }
        #endif
        
        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
        {
            if (LidarDrv.recvData ((char*)&(temp_ultra_capsule_node.s_checksum_1), wait_size, &data_buffer) != wait_size)
                continue;
            
            temp_byte = temp_ultra_capsule_node.s_checksum_1;
          
            if ((temp_byte >> 4) == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1)
            {
                wait_size = size_b;
                
                if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                {
                    if (LidarDrv.recvData ((char*)&(temp_ultra_capsule_node.s_checksum_2), wait_size, &data_buffer) != wait_size)
                        continue;
                    
                    temp_byte = temp_ultra_capsule_node.s_checksum_2;
                     
                    if ((temp_byte >> 4) == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2)
                    {
                        wait_size = size_c;
                        
                        if (LidarDrv.waitData (wait_size, &wait_time_ms, &data_buffer) == TRUE)
                        {
                            #ifdef _CALC_DRIVER_PROCESS_TIME_
                            if (wait_time_ms < 500)
                            {
                                wait_cost_time_ms += 500 - wait_time_ms;
                            }
                            #endif
                          
                            if (LidarDrv.recvData ((char*)&(temp_ultra_capsule_node.start_angle_sync_q6), wait_size, &data_buffer) != wait_size)
                                continue;
                            
                            recv_check_sum = ((temp_ultra_capsule_node.s_checksum_1 & 0xF) | (temp_ultra_capsule_node.s_checksum_2 << 4));
                            
                            check_sum = 0;    //check_sum参与异或运算前先恢复至0，避免以前残留的值参与运算；
                            
                            for (unsigned int i = 0; i < wait_size; i++)
                            {
                                check_sum ^= ((unsigned char*)&(temp_ultra_capsule_node.start_angle_sync_q6))[i];
                            }

                            if (check_sum == recv_check_sum)
                            {
                                if ((temp_ultra_capsule_node.start_angle_sync_q6) & (RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT))         //若该位为1，表明是一次全新的扫描过程（注意：不是新的一圈）；
                                {
                                    is_previous_capsuledataRdy = FALSE;
        
                                    hq_node_count = 0;
                                }
                                
                                if (is_previous_capsuledataRdy == TRUE)
                                {
                                    ultraCapsuleToNormal (&previous_ultra_capsule_node, &temp_ultra_capsule_node, temp_hq_buf, &temp_node_count);
                                }
                                
                                previous_ultra_capsule_node = temp_ultra_capsule_node;
                                    
                                is_previous_capsuledataRdy = TRUE;
                                
                                temp_count = temp_node_count;
                                
                                while(temp_node_count > 0)    
                                {
                                    if (((LidarDrv.bufferState_A == IS_EMPTY) || (LidarDrv.bufferState_A == IS_WRITING)) && (LidarDrv.bufferState_B != IS_WRITING) && (temp_node_count > 0))
                                    {
                                        LidarDrv.bufferState_A = IS_WRITING;
                                    
                                        unsigned int j = 0;
                                    
                                        if (temp_node_count < temp_count)
                                        {
                                            j = temp_count - temp_node_count;
                                        }
                                    
                                        for ( ; j < temp_count; j++)
                                        {
                                            LidarDrv.nodeBuffer_A[hq_node_count] = temp_hq_buf[j];
                                        
                                            hq_node_count++;
                                        
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_hq_buf[j].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) //当temp_hq_buf[j].flag第一次为1的时候，表明已经收到了第一圈数据（一般是不完整的）；                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                                
                                                    hq_node_count = 0;                                  //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                                
                                                    continue;
                                                }
                                            
                                                LidarDrv.validNodePos_A = hq_node_count;
                                            
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_B != IS_READING)    //如果buffer_B没有被读取，那么应当将buffer_B置为IS_EMPTY，后续数据应当向buffer_B中存储；   
                                                {
                                                    LidarDrv.bufferState_B = IS_EMPTY;
                                            
                                                    LidarDrv.bufferState_A = IS_READY;

                                                    LidarDrv.ready_time_ms_A = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_A));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_A * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                        
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);
                                                    
                                                        wait_cost_time_ms = 0;
                                                        
                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                        
                                                    j++;         //j++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_B正在被读取，那么后续数据应当继续向当前的buffer_A中存储；
                                                }
                                            }
                                        }
                                        
                                        temp_node_count = temp_count - j;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则j==temp_count，那么此处就是0；
                                    }
                                
                                    if (((LidarDrv.bufferState_B == IS_EMPTY) || (LidarDrv.bufferState_B == IS_WRITING)) && (LidarDrv.bufferState_A != IS_WRITING) && (temp_node_count > 0))
                                    {
                                        LidarDrv.bufferState_B = IS_WRITING;
                                    
                                        unsigned int k = 0;
                                    
                                        if (temp_node_count < temp_count)
                                        {
                                            k = temp_count - temp_node_count;
                                        }
                                    
                                        for ( ; k < temp_count; k++)
                                        {
                                            LidarDrv.nodeBuffer_B[hq_node_count] = temp_hq_buf[k]; 
                                        
                                            hq_node_count++;
                                        
                                            if (hq_node_count >= MAX_SCAN_NODES)
                                            {
                                                hq_node_count = 0;
                                            }
                                        
                                            if (temp_hq_buf[k].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)                    
                                            {
                                                if (first_circle_come == FALSE)
                                                {
                                                    first_circle_come = TRUE;
                                    
                                                    hq_node_count = 0;                       //丢弃第一圈数据，continue表示继续接收剩余的几个node，不能用break，break会丢弃剩余的几个node；
                                                
                                                    continue;
                                                }
                                            
                                                LidarDrv.validNodePos_B = hq_node_count;
                                            
                                                hq_node_count = 0;
                                            
                                                if (LidarDrv.bufferState_A != IS_READING)    //如果buffer_A没有被读取，那么应当将buffer_A置为IS_EMPTY，后续数据应当向buffer_A中存储；   
                                                {
                                                    LidarDrv.bufferState_A = IS_EMPTY;
                                            
                                                    LidarDrv.bufferState_B = IS_READY;
  
                                                    LidarDrv.ready_time_ms_B = data_buffer->start_DMA_time_ms + (data_buffer->finish_DMA_time_ms - data_buffer->start_DMA_time_ms) * data_buffer->readPos / data_buffer->validPos;
                                                    
                                                    notify_data_ready_event (&(LidarDrv.bufferState_B));
                                                    
                                                    #ifdef _CALC_DRIVER_PROCESS_TIME_
                                                        one_frame_finish_time_ms = getms ();    //获取系统当前时间；
                                                    
                                                        frame_interval_ms = LidarDrv.validNodePos_B * (LidarDrv.ScanMode[LidarDrv.currentScanModeID].us_per_sample) / 1000;

                                                        one_frame_process_time_ms = one_frame_finish_time_ms - one_frame_start_time_ms - wait_cost_time_ms;
                                                        
                                                        printf ("frame_interval_ms = %d\none_frame_process_time_ms = %d\n\n", frame_interval_ms, one_frame_process_time_ms);
                                                    
                                                        wait_cost_time_ms = 0;
                                                        
                                                        one_frame_come_flag = FALSE;
                                                    #endif
                                                        
                                                    k++;          //k++是为了避免当前已被转存的node被重复转存；
                                            
                                                    break;
                                                }
                                                else
                                                {
                                                    continue;    //如果buffer_A正在被读取，那么后续数据应当继续向当前的buffer_B中存储；
                                                }
                                            }
                                        }
                                        
                                        temp_node_count = temp_count - k;    //如果还剩余有node未被转存，要更新temp_node_count的数值，以备后续使用，如果for循环正常退出，则k==temp_count，那么此处就是0；
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    LidarDrv.currentStatus = IS_STOPPED;
    
    return;
}

/*转换CapsuledScanData到HQ格式，并存入hq_buf*/
static void capsuleToNormal(rplidar_response_capsule_measurement_nodes_t *previous_capsule, const rplidar_response_capsule_measurement_nodes_t *capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count)
{    
    if ((previous_capsule == NULL) || (capsule == NULL) || (hq_buf == NULL))
    {
        return;
    }
    
    unsigned int  temp_count = 0;
    
    rplidar_response_measurement_node_hq_t  node;

    int diffAngle_q8;
    int currentStartAngle_q8 = ((capsule->start_angle_sync_q6 & 0x7FFF) << 2);
    int prevStartAngle_q8 = ((previous_capsule->start_angle_sync_q6 & 0x7FFF) << 2);

    diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
    if (prevStartAngle_q8 > currentStartAngle_q8) 
    {
        diffAngle_q8 += (360<<8);
    }

    int angleInc_q16 = (diffAngle_q8 << 3);
    int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
    for (unsigned int pos = 0; pos < 16; ++pos)
    {
        int dist_q2[2];
        int angle_q6[2];
        int syncBit[2];

        dist_q2[0] = (previous_capsule->cabins[pos].distance_angle_1 & 0xFFFC);
        dist_q2[1] = (previous_capsule->cabins[pos].distance_angle_2 & 0xFFFC);

        int angle_offset1_q3 = ((previous_capsule->cabins[pos].offset_angles_q3 & 0xF) | ((previous_capsule->cabins[pos].distance_angle_1 & 0x3)<<4));
        int angle_offset2_q3 = ((previous_capsule->cabins[pos].offset_angles_q3 >>  4) | ((previous_capsule->cabins[pos].distance_angle_2 & 0x3)<<4));

        angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
        syncBit[0] =  (((currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 ) ? 1 : 0;
        currentAngle_raw_q16 += angleInc_q16;

        angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
        syncBit[1] =  (((currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 ) ? 1 : 0;
        currentAngle_raw_q16 += angleInc_q16;

        for (unsigned int cpos = 0; cpos < 2; ++cpos) 
        {
            if (angle_q6[cpos] < 0) 
                angle_q6[cpos] += (360<<6);
                    
            if (angle_q6[cpos] >= (360<<6)) 
                angle_q6[cpos] -= (360<<6);

            node.angle_z_q14 = (unsigned short)((angle_q6[cpos] << 8) / 90);
            node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
            node.quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            node.dist_mm_q2 = dist_q2[cpos];
                
            hq_buf[temp_count] = node;
            temp_count++; 
        }
    }
    
    *node_count = temp_count; 
    
    return;    
}

static unsigned int _varbitscale_decode(unsigned int scaled, unsigned int* scaleLevel)
{
    static const unsigned int VBS_SCALED_BASE[] = 
    {
        RPLIDAR_VARBITSCALE_X16_DEST_VAL,
        RPLIDAR_VARBITSCALE_X8_DEST_VAL,
        RPLIDAR_VARBITSCALE_X4_DEST_VAL,
        RPLIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const unsigned int VBS_SCALED_LVL[] = 
    {
        4,
        3,
        2,
        1,
        0,
    };

    static const unsigned int VBS_TARGET_BASE[] = 
    {
        (0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (unsigned int i = 0; i < 5; ++i)
    {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) 
        {
            *scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << *scaleLevel);
        }
    }
    
    return 0;
}

/*转换UltraCapsuledScanData到HQ格式，并存入hq_buf*/
static void ultraCapsuleToNormal(rplidar_response_ultra_capsule_measurement_nodes_t *previous_ultra_capsule, const rplidar_response_ultra_capsule_measurement_nodes_t *ultra_capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count)
{
    if ((previous_ultra_capsule == NULL) || (ultra_capsule == NULL) || (hq_buf == NULL))
    {
        return;
    }
    
    unsigned int temp_count = 0;
    
    rplidar_response_measurement_node_hq_t  node;

    int diffAngle_q8;
    int currentStartAngle_q8 = ((ultra_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
    int prevStartAngle_q8 = ((previous_ultra_capsule->start_angle_sync_q6 & 0x7FFF) << 2);

    diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
    if (prevStartAngle_q8 > currentStartAngle_q8) 
    {
        diffAngle_q8 += (360 << 8);
    }

    int angleInc_q16 = (diffAngle_q8 << 3) / 3;
    int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
    for (unsigned int pos = 0; pos < 32; ++pos)
    {
        int dist_q2[3];
        int angle_q6[3];
        int syncBit[3];

        unsigned int combined_x3 = previous_ultra_capsule->ultra_cabins[pos].combined_x3;

        // unpack ...
        int dist_major = (combined_x3 & 0xFFF);

        // signed partical integer, using the magic shift here
        // DO NOT TOUCH

        int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
        int dist_predict2 = (((int)combined_x3) >> 22);

        int dist_major2;

        unsigned int scalelvl1, scalelvl2;

        // prefetch next ...
        if (pos == (32 - 1))
        {
            dist_major2 = (ultra_capsule->ultra_cabins[0].combined_x3 & 0xFFF);
        }
        else 
        {
            dist_major2 = (previous_ultra_capsule->ultra_cabins[pos + 1].combined_x3 & 0xFFF);
        }

        // decode with the var bit scale ...
        dist_major = _varbitscale_decode(dist_major, &scalelvl1);
        dist_major2 = _varbitscale_decode(dist_major2, &scalelvl2);

        int dist_base1 = dist_major;
        int dist_base2 = dist_major2;

        if ((!dist_major) && dist_major2) 
        {
            dist_base1 = dist_major2;
            scalelvl1 = scalelvl2;
        }
          
        dist_q2[0] = (dist_major << 2);
        if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) 
        {
            dist_q2[1] = 0;
        } 
        else 
        {
            dist_predict1 = (dist_predict1 << scalelvl1);
            dist_q2[1] = (dist_predict1 + dist_base1) << 2;
        }

        if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) 
        {
            dist_q2[2] = 0;
        } 
        else 
        {
            dist_predict2 = (dist_predict2 << scalelvl2);
            dist_q2[2] = (dist_predict2 + dist_base2) << 2;
        }
           
        for (int cpos = 0; cpos < 3; ++cpos)
        {
            syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

            int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

            if (dist_q2[cpos] >= (50 * 4))
            {
                const int k1 = 98361;
                const int k2 = (int)(k1 / dist_q2[cpos]);

                offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
            }

            angle_q6[cpos] = ((currentAngle_raw_q16 - (int)(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
            if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

            node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
            node.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            node.angle_z_q14 = (unsigned short)((angle_q6[cpos] << 8) / 90);
            node.dist_mm_q2 = dist_q2[cpos];

            hq_buf[temp_count] = node;
            temp_count++; 
        }
    }
    
    *node_count = temp_count; 
    
    return;
}

/*转换DenseCapsuledScanData到HQ格式，并存入hq_buf*/
static void dense_capsuleToNormal (rplidar_response_capsule_measurement_nodes_t *previous_capsule, const rplidar_response_capsule_measurement_nodes_t *capsule, rplidar_response_measurement_node_hq_t *hq_buf, int *node_count)
{
    if ((previous_capsule == NULL) || (capsule == NULL) || (hq_buf == NULL))
    {
        return;
    }
    
    unsigned int  temp_count = 0;
    
    rplidar_response_measurement_node_hq_t  node;
    
    rplidar_response_dense_capsule_measurement_nodes_t *previous_dense_capsule = (rplidar_response_dense_capsule_measurement_nodes_t*)previous_capsule;
    const rplidar_response_dense_capsule_measurement_nodes_t *dense_capsule = (rplidar_response_dense_capsule_measurement_nodes_t*)capsule;

    int diffAngle_q8;
    int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
    int prevStartAngle_q8 = ((previous_dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);

    diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
    if (prevStartAngle_q8 > currentStartAngle_q8) 
    {
        diffAngle_q8 += (360 << 8);
    }

    int angleInc_q16 = (diffAngle_q8 << 8)/40;
    int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
    for (unsigned int pos = 0; pos < 40; ++pos)
    {
        int dist_q2;
        int angle_q6;
        int syncBit;
        const int dist = (const int)(previous_dense_capsule->cabins[pos].distance);
        dist_q2 = dist << 2;
        angle_q6 = (currentAngle_raw_q16 >> 10);
        syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;
        currentAngle_raw_q16 += angleInc_q16;

        if (angle_q6 < 0) angle_q6 += (360 << 6);
        if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);

        node.angle_z_q14 = (unsigned short)((angle_q6 << 8) / 90);
        node.flag = (syncBit | ((!syncBit) << 1));
        node.quality = dist_q2 ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
        node.dist_mm_q2 = dist_q2;
                
        hq_buf[temp_count] = node;
        temp_count++; 
    }
    
    *node_count = temp_count; 
    
    return;  
}

/*获取标准格式的雷达数据*/
static bool _grabScanDataHq (rplidar_response_measurement_node_hq_t *nodebuffer, unsigned int *count, unsigned int time_out_ms)
{
    if ((nodebuffer == NULL) || (count == NULL))
        return FALSE;
    
    if (LidarDrv.bufferState_A == IS_READY)
    {
        LidarDrv.bufferState_A = IS_READING;
  
        for (unsigned int i = 0; i < LidarDrv.validNodePos_A; i++)
        {
            nodebuffer[i] = LidarDrv.nodeBuffer_A[i];        
        }
        
        *count = LidarDrv.validNodePos_A;
        
        LidarDrv.bufferState_A = IS_EMPTY;
        
        return TRUE;      
    }
    else if (LidarDrv.bufferState_B == IS_READY)
    {
        LidarDrv.bufferState_B = IS_READING;
  
        for (unsigned int j = 0; j < LidarDrv.validNodePos_B; j++)
        {
            nodebuffer[j] = LidarDrv.nodeBuffer_B[j];        
        }
        
        *count = LidarDrv.validNodePos_B;
        
        LidarDrv.bufferState_B = IS_EMPTY;
        
        return TRUE;      
    }
    
    return FALSE;
}

/*通知一圈数据已经准备好*/
static void notify_data_ready_event (void* arg)
{
    view_dot_task (arg);
}









