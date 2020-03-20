/*
 *  RPLIDAR STM32 Driver
 * 
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

1. 本工程是在STM32 CubeMX中选中STM32F401RE_Nucleo开发板，然后配置所需外设（UART1、UART1 Rx的DMA、UART2、UART6、PWM）后生成的IAR7.6 Project；

2. STM32F401RE_Nucleo开发板背面的SB62、SB63各自连通后开发板正面CN9上面的D0、D1才能和UART2的Rx、Tx连通；

3. RPLidar通过UART1和STM32F401双向通信；

4. main函数中在与雷达通信前可对UART1、UART2、UART6的波特率进行设置（115200、256000）；

5. 工程中的printf函数绑定至UART2，可用于向外打印信息；

6. UART6可供用户自由使用；

7. 本程序的基本思路是——：定义一个FIFO buffer，作为UART1 Rx的DMA缓冲区，DMA接收将雷达数据填充到缓冲区，雷达数据解析函数从缓冲区中读取数据；

8. 雷达驱动程序准备好一圈数据后会触发函数void view_dot_task (void* arg); 用户希望对一圈数据进行的操作请在该函数中完成 ；

9. 准备好的一圈数据存储于LidarDrv.nodeBuffer_A[]或者LidarDrv.nodeBuffer_B[]中，存储的数量由LidarDrv.validNodePos_A和LidarDrv.validNodePos_B表示，对于quality值为0点请勿使用；

10. main.h中的 #define _CALC_DRIVER_PROCESS_TIME_   用于使能进行驱动解析耗时的计算；

11. 本程序支持RPLidar A1M1-R1 ———— S1M1-R1;


关于UART1 Rx Single DMA FIFO Buffer——：
  1. 建立一个一维字节数组uart1_rx_DMA_buffer，数组大小为 UART1_RX_BUFFER_SIZE * UART1_RX_BUFFER_AMOUNT；

  2. uart1_rx_DMA_buffer所占用的连续的内存空间从逻辑上可分为UART1_RX_BUFFER_AMOUNT份；

  3. 建立一个一维结构体数组uart1_rx_buffer，数组大小为UART1_RX_BUFFER_AMOUNT；

  4. uart1_rx_buffer中的每一个的元素中的data指针依次指向uart1_rx_DMA_buffer的第n份的首地址；

  5. 将uart1_rx_buffer构建为一个先进先出环形队列（FIFO）；

  6. Ring_Enqueue函数只向外反馈环形队列是否可进行入列操作，以及可进行入列操作的buffer的地址，不执行实际的入列操作；

  7. Ring_Dequeue函数只向外反馈环形队列是否可进行出列操作，以及可进行出列操作的buffer的地址，不执行实际的出列操作；

  8. UART1 DMA接收过程（不占用CPU时间）完成真正的数据入列（整存），并在接收完成中断函数中调整队列尾巴，完成入列操作；

  9. Recv_chars函数从环形队列中读取数据（零取），当传入的buffer被读空时，调整队列头，完成出列操作；
  
  10. “整存零取”——：指的是一次一次的DMA接收只能一次一次按照开启DMA时设定的size存size个字节进缓冲区，而Recv_chars函数则能够自由地从缓冲区读“任意数量”的字节（“任意数量” ≤ 缓冲区可读字节数量）；


关于uart1_rx_DMA_ready_size --;的原子操作——：
  1. 全局变量uart1_rx_DMA_ready_size已用volatile修饰，CPU每次对其进行操作时将其直接从内存中读到寄存器，再进行操作；

  2. uart1_rx_DMA_ready_size --;这条语句执行的过程是——：
     a. CPU将uart1_rx_DMA_ready_size从内存中读到寄存器R1；
     b. 寄存器R1减1；
     c. 将寄存器R1回写到内存；

  3. 主程序每读取一个字节都会令uart1_rx_DMA_ready_size减减，因此操作uart1_rx_DMA_ready_size的频次是很高的；

  4. 每次UART1 Rx DMA接收中断到来也会操作uart1_rx_DMA_ready_size，刷新其值，假如uart1_rx_DMA_ready_size --;不是原子操作，那么便会概率性出现如下情况——：
     a. CPU将uart1_rx_DMA_ready_size从内存中读到寄存器R1（假定读到的值是120）；
     b. UART1 Rx DMA中断到来，操作被打断，中断函数中读取并刷新uart1_rx_DMA_ready_size的值，然后回写到内存（假定此时内存中的值变为120+64=188）；
     c. 寄存器R1减1（结果为119）；
     d. 将寄存器R1回写到内存（内存中的188被擦除，重写为119）；

  5. 实践证明，如果uart1_rx_DMA_ready_size --;不是原子操作，“4.”中的情况便无法避免，uart1_rx_DMA_ready_size的值会错乱，调整为原子操作后，uart1_rx_DMA_ready_size的值便能持续保持准确；
   