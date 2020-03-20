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
#include "net_socket.h"
#include "timer.h"


/*新增TCP_Clear_rx函数，仅供雷达驱动调用*/
void TCP_Clear_rx(void)
{

}

/*新增TCP_Clear_tx函数，仅供雷达驱动调用*/
void TCP_Clear_tx(void)
{

}

/*新增Recv_chars函数，仅供雷达驱动调用*/
int TCP_Recv_chars(char *addr, int length, UART1_RX_BUFFER** src_buffer)
{
	int bytes_read = 0;

	return bytes_read;
}

/*新增TCP_Wait_chars函数，仅供雷达驱动调用*/
bool TCP_Wait_chars(int wait_size, unsigned int *time_out_ms, UART1_RX_BUFFER **src_buffer)
{

    return FALSE;
}


int TCP_Send_chars(char *string, int length)
{
	int bytes_sent = 0;

	return bytes_sent;
}


bool Connect_TCP_server(void)
{

	return FALSE;
}


void Close_TCP_connect(void)
{

}


bool Remote_Set_dtr_signal(bool ctrl)
{

    return TRUE;
}            


bool Remote_Set_rts_signal(bool ctrl)
{	
	
	return TRUE;
}




