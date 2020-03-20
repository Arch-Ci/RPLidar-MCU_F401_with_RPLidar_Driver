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

#ifndef UART_DMA_H_
#define UART_DMA_H_
#include "rplidar.h"

#define    UART1_RX_BUFFER_SIZE     64
#define    UART1_RX_BUFFER_AMOUNT   80

typedef enum 
{
  FALSE = 0,
  TRUE = 1,
} bool;

/*枚举buffer的状态*/
typedef enum
{
    IS_EMPTY = 0,
    IS_WRITING,
    IS_READING,
    IS_READY, 
} enum_buffer_state;

typedef struct _uart1_rx_dma_buffer
{
    enum_buffer_state state;           //用于表征UART1 Rx的DMA缓冲区的状态
    
    unsigned int start_DMA_time_ms;    //开始DMA接收的时间
    
    unsigned int finish_DMA_time_ms;   //完成DMA接收的时间
    
    unsigned int validPos;             //用于表征UART1 Rx的DMA缓冲区中多少数据就绪
    
    unsigned int readPos;              //用于表征UART1 Rx的DMA缓冲区中接下来应被读取的数据的下标

    char* data;                        //UART1 Rx的DMA缓冲区
    
} UART1_RX_BUFFER;

typedef struct _ring_buffer
{
    UART1_RX_BUFFER *p_buffer;
    unsigned int head; 
    unsigned int tail; 
    unsigned int buffer_size;
    
} RING_BUFFER;

extern unsigned int prev_wait_size;
extern unsigned int curr_wait_size;
extern unsigned int next_wait_size;
extern RING_BUFFER uart1_rx_ring_buffer;
extern volatile unsigned int uart1_rx_DMA_ready_size;
extern UART1_RX_BUFFER *curr_using_buffer;
extern bool no_buffer_could_use_flag;

void uart1_rx_buffer_init (void);

void Ring_Buffer_Init (void);

bool Ring_Enqueue (UART1_RX_BUFFER** p_temp, RING_BUFFER *p_ring);
bool Ring_Dequeue (UART1_RX_BUFFER** p_temp, RING_BUFFER *p_ring);

UART1_RX_BUFFER* add_uart1_rx_task (unsigned int size);
UART1_RX_BUFFER* get_curr_ready_uart1_rx_buffer_addr (void);

#endif
