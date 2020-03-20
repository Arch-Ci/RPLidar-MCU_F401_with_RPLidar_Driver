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
#include "uart_dma.h"
#include "timer.h"

unsigned int prev_wait_size = 0;      //���ڱ����ϴ�ϣ���ȴ����ֽ�����
unsigned int curr_wait_size = 0;      //���ڱ�����ǰϣ���ȴ����ֽ�����
unsigned int next_wait_size = 0;      //���ڱ����´�ϣ���ȴ����ֽ�����

static unsigned int prev_DMA_time_ms = 0;    //���ڼ�¼�ϴ�DMA�жϵ�����ʱ��
static unsigned int curr_DMA_time_ms = 0;    //���ڼ�¼����DMA�жϵ�����ʱ��

static char uart1_rx_DMA_buffer[UART1_RX_BUFFER_SIZE*UART1_RX_BUFFER_AMOUNT];

static UART1_RX_BUFFER uart1_rx_buffer[UART1_RX_BUFFER_AMOUNT];

RING_BUFFER uart1_rx_ring_buffer;

UART1_RX_BUFFER *curr_using_buffer = NULL;
UART1_RX_BUFFER *next_using_buffer = NULL;

bool no_buffer_could_use_flag = FALSE;

volatile unsigned int uart1_rx_DMA_ready_size = 0;


/*uart1 rx DMA buffer��ʼ��*/
void uart1_rx_buffer_init (void)
{ 
    for (unsigned int i = 0; i < UART1_RX_BUFFER_AMOUNT; i++)
    {  
        uart1_rx_buffer[i].state = IS_EMPTY;
        uart1_rx_buffer[i].start_DMA_time_ms = 0;
        uart1_rx_buffer[i].finish_DMA_time_ms = 0;
        uart1_rx_buffer[i].validPos = 0;
        uart1_rx_buffer[i].readPos = 0;
        uart1_rx_buffer[i].data = &uart1_rx_DMA_buffer[i*UART1_RX_BUFFER_SIZE];
        
        for (unsigned int j = 0; j < UART1_RX_BUFFER_SIZE; j++)
        {
            uart1_rx_buffer[i].data[j] = 0;
        }
    }
}

/*  
* @fuction:      Ring_Buffer_Init
* @brief:        ���ζ��г�ʼ�� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ���ζ��г�ʼ��ʱʹ�� 
*/
void Ring_Buffer_Init (void)
{
    uart1_rx_ring_buffer.p_buffer = uart1_rx_buffer;
    uart1_rx_ring_buffer.head = 0;
    uart1_rx_ring_buffer.tail = 0;
    uart1_rx_ring_buffer.buffer_size = UART1_RX_BUFFER_AMOUNT - 1;			  	//buffer_size����Ϊʵ�ʴ�С - 1
}

/*  
* @fuction:      Ring_Enqueue
* @brief:        ���ⲿ���������п��Խ������в�����buffer�ĵ�ַ
* @param[in]:    *p_temp: �洢�����п��Խ������в�����buffer�ĵ�ַ, p_ring:���ζ��еĵ�ַ
* @param[out]:   �����п��Խ������в�����buffer�ĵ�ַ
* @return:       FALSE:ʧ��, TRUE:�ɹ�
* @others:        
*/
bool Ring_Enqueue (UART1_RX_BUFFER** p_temp, RING_BUFFER *p_ring)
{
    if(((p_ring->tail+1) == p_ring->head) || ((p_ring->tail == p_ring->buffer_size) && (p_ring->head==0)))		
    {
		*p_temp = NULL;
        return FALSE;
    }
    else
    {
        *p_temp = &(p_ring->p_buffer[p_ring->tail]);                    
        //p_ring->tail++;								               
        //if(p_ring->tail > p_ring->buffer_size)	//���������в�����DMA�����ж�����ɣ�������������ɺ�����޸Ķ���β��	               
        //    p_ring->tail = 0;							           
        return TRUE;
    }
}

/*  
* @fuction:      Ring_Dequeue
* @brief:        ���ⲿ���������п��Խ��г��в�����buffer�ĵ�ַ
* @param[in]:    *p_temp: �洢�����п��Խ��г��в�����buffer�ĵ�ַ, p_ring:���ζ��еĵ�ַ
* @param[out]:   �����п��Խ��г��в�����buffer�ĵ�ַ
* @return:       FALSE:ʧ��, TRUE:�ɹ�
* @others:        
*/
bool Ring_Dequeue (UART1_RX_BUFFER** p_temp, RING_BUFFER *p_ring)
{   
    if(p_ring->head == p_ring->tail)					          
    {     
        *p_temp = NULL;
        return FALSE;                                        	 
    }
    else
    {
        *p_temp = &(p_ring->p_buffer[p_ring->head]);
        //p_ring->head++;							            	  
        //if(p_ring->head > p_ring->buffer_size)	//�����ĳ��в�����serial.c�е�Recv_chars()��������ɣ������ĳ�����ɺ�����޸Ķ���ͷ	          
        //    p_ring->head = 0;                          
        return TRUE;
    }   
} 

/*���uart1 rx DMA��������*/
UART1_RX_BUFFER* add_uart1_rx_task (unsigned int size)
{
    UART1_RX_BUFFER* curr_recv_buffer = NULL;
    
    if (Ring_Enqueue(&curr_recv_buffer, &uart1_rx_ring_buffer) == TRUE)
    {
        HAL_UART_Receive_DMA(&huart1, curr_recv_buffer->data, size);

        curr_recv_buffer->state = IS_WRITING;
        curr_recv_buffer->start_DMA_time_ms = getms ();
    }
    
    return curr_recv_buffer;
}

/*��ȡ��ǰ�ɶ���uart1 rx DMA�������ĵ�ַ*/
UART1_RX_BUFFER* get_curr_ready_uart1_rx_buffer_addr (void)
{
    UART1_RX_BUFFER* ready_addr = NULL;
    
    if (Ring_Dequeue(&ready_addr, &uart1_rx_ring_buffer) == TRUE)
    {
        return ready_addr;
    }
    
    return ready_addr;
}

/*UART DMA�����жϴ�����*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
    prev_DMA_time_ms = curr_DMA_time_ms;      
    curr_DMA_time_ms = getms();
  
    if (next_using_buffer != NULL)
    {
        curr_using_buffer = next_using_buffer;
    }
    
    if (curr_using_buffer->state == IS_WRITING)
    {
        curr_using_buffer->state = IS_READY;
        
        curr_using_buffer->finish_DMA_time_ms = curr_DMA_time_ms;
      
        curr_using_buffer->validPos = curr_wait_size;
        
        curr_using_buffer->readPos = 0;
        
        uart1_rx_DMA_ready_size += curr_wait_size;        //����ȫ��ready size
       
        /*******�������,��������*******/
        uart1_rx_ring_buffer.tail++;       
        if (uart1_rx_ring_buffer.tail > uart1_rx_ring_buffer.buffer_size)		               
            uart1_rx_ring_buffer.tail = 0;
        /*******************************/         
        
        if (next_wait_size > 0)
        {
            next_using_buffer = add_uart1_rx_task(next_wait_size);
          
            if (next_using_buffer == NULL)
            {
                no_buffer_could_use_flag = TRUE;
            }
            else
            {
                curr_wait_size = next_wait_size;
            }
        } 
        else
        {
            next_using_buffer = NULL; 
        }
    }
}
