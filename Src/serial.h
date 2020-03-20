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

#ifndef SERIE_H_
#define SERIE_H_
#include "uart_dma.h"

void Clear_rx(void);                        /*新增Clear_rx函数，仅供雷达驱动调用*/

void Clear_tx(void);                        /*新增Clear_tx函数，仅供雷达驱动调用*/

int Recv_chars(char *addr, int length, UART1_RX_BUFFER **src_buffer);/*新增Recv_chars函数，仅供雷达驱动调用*/

bool Wait_chars(int wait_size, unsigned int *time_out_ms, UART1_RX_BUFFER **src_buffer);/*新增Wait_chars函数，仅供雷达驱动调用*/

int Send_chars(char *, int);

bool Config_port(void);

bool Set_dtr_signal(bool);

bool Set_rts_signal(bool);

void Close_port(void);

#endif
