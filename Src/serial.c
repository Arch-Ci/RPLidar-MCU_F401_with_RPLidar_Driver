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
#include "rplidar.h"
#include "serial.h"
#include "timer.h"


/*新增Clear_rx函数，仅供雷达驱动调用*/
void Clear_rx(void)
{
    uart1_rx_buffer_init ();   //UART1 Rx DMA buffer初始化
  
    Ring_Buffer_Init ();       //环形队列初始化
}

/*新增Clear_tx函数，仅供雷达驱动调用*/
void Clear_tx(void)
{

}


/*新增Recv_chars函数，仅供雷达驱动调用*/
int Recv_chars(char *addr, int length, UART1_RX_BUFFER **src_buffer)
{
	unsigned int i = 0, j = 0;
    
    if ((addr == NULL) || (*src_buffer == NULL) || (length <= 0))
    {
        return 0;
    }
    
    (*src_buffer)->state = IS_READING;
    
    do
    {
        for (i = j; i < length; i++)
        {
            addr[i] = (*src_buffer)->data[(*src_buffer)->readPos];
            
            (*src_buffer)->readPos ++;  
           
            unsigned int context = enter_critical_section ();    //重要：进入临界区！！！
            
            uart1_rx_DMA_ready_size --;    //递减全局ready size
            
            leave_critical_section (context);                    //重要：退出临界区，挂起的中断允许继续触发！！！
            
            if ((*src_buffer)->readPos >= (*src_buffer)->validPos)
            {
                (*src_buffer)->state = IS_EMPTY;
                
                /*******出列完成,调整队列*******/
                uart1_rx_ring_buffer.head++;							            	  
                if(uart1_rx_ring_buffer.head > uart1_rx_ring_buffer.buffer_size)		 	          
                    uart1_rx_ring_buffer.head = 0;
                /*******************************/
                
                if (i+1 < length)
                {
                    *src_buffer = get_curr_ready_uart1_rx_buffer_addr();
                
                    if (*src_buffer == NULL)
                    {
                        return i++;
                    }
                    
                    (*src_buffer)->state = IS_READING;
                }
                
                i++;
                  
                break;  
            }
        }
        
        j = i;
        
    } while(i < length);
    
	return i;
}


/*新增Wait_chars函数，仅供雷达驱动调用*/
bool Wait_chars(int wait_size, unsigned int *time_out_ms, UART1_RX_BUFFER **src_buffer)
{
    unsigned int   start_time_ms;
    unsigned int   elapsed_time_ms;

    UART1_RX_BUFFER* now_ready_buffer = NULL;

    start_time_ms = getms ();           //获取系统当前时间；
    elapsed_time_ms = 0;                //本函数消耗掉的时间 
    
    if(wait_size <= 0)
		return FALSE;

    if (prev_wait_size == 0)
    {
        curr_using_buffer = add_uart1_rx_task (wait_size);
    }     

    while (elapsed_time_ms < *time_out_ms) 
    {
        if (uart1_rx_DMA_ready_size >= wait_size)
        {
            now_ready_buffer = get_curr_ready_uart1_rx_buffer_addr();
        }
        else if (no_buffer_could_use_flag == TRUE)
        {
            curr_using_buffer = add_uart1_rx_task(next_wait_size);
          
            if (curr_using_buffer != NULL) 
            {
                no_buffer_could_use_flag = FALSE; 
            }
        }
        
        if (now_ready_buffer != NULL)
        {
            *src_buffer = now_ready_buffer;  
          
            elapsed_time_ms = getms() - start_time_ms;
        
            *time_out_ms -= elapsed_time_ms;
        
            return TRUE;
        }
  
        elapsed_time_ms = getms() - start_time_ms;
    }    

    return FALSE;
}

/*发送串口数据函数，仅供雷达驱动调用*/
int Send_chars(char *string, int length)
{
    if (HAL_UART_Transmit(&huart1, (uint8_t*)string, length, 0xFFFF) == HAL_OK)
    {
        return length;
    }
    
	return 0;
}

bool Config_port(void)
{
  
	return TRUE;
}


void Close_port(void)
{

}

bool Set_dtr_signal(bool ctrl)
{

    return TRUE;
}            

bool Set_rts_signal(bool ctrl)
{	

	return TRUE;
}


