/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "serial.h"
#include "net_socket.h"
#include "task.h"
#include "timer.h"
#include "uart_dma.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(unsigned short pwm);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);    //��stdio.h�е�printf��������UART2
    return ch;
}

#define cli __disable_irq
#define sei __enable_irq

/*�����ٽ���*/
inline unsigned int enter_critical_section(void)
{
    unsigned int context=__get_PRIMASK();
    cli();
    return context;
}

/*�˳��ٽ���*/
inline void leave_critical_section(unsigned int context)
{
    __set_PRIMASK(context);
}

unsigned int lidar_baudrate = 256000;

HAL_StatusTypeDef status;

/*�趨pwmֵ�������״�ת�٣���Χ��0-MAX_MOTOR_PWM*/
void MX_TIM3_Set_Pwm(unsigned short pwm)
{
    if (pwm > MAX_MOTOR_PWM)
    {
        pwm = MAX_MOTOR_PWM;
    }
    
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  
    MX_TIM3_Init (pwm);
  
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/*�û�ϣ���Եײ�����׼���õ�����һȦ���������Ĳ��������ڴ˺����ڽ���*/
void view_dot_task (void* arg)
{
    if ((enum_buffer_state*)arg == &(LidarDrv.bufferState_A))
    {
        LidarDrv.bufferState_A = IS_READING;
      
        /*�û�ϣ����һȦ���ݽ��еĲ��������������*/
      
        printf ("LidarDrv.validNodePos_A = %d\nLidarDrv.ready_time_ms_A = %d\n", LidarDrv.validNodePos_A, LidarDrv.ready_time_ms_A);
        HAL_Delay (80);    //����ʱ����ģ�⼫�޸��ɴ���ʱ�䣬���ʱ��ɾ��
        /*for (unsigned int i = 0; i < LidarDrv.validNodePos_A; i++)
        {
            if (LidarDrv.nodeBuffer_A[i].quality > 0)
            {
                printf ("A = %.2f  D = %d\n", LidarDrv.nodeBuffer_A[i].angle_z_q14 * 90.f / (1 << 14), LidarDrv.nodeBuffer_A[i].dist_mm_q2 / (1 << 2));
            }
        }*/ 
      
        /*�û�ϣ����һȦ���ݽ��еĲ��������������*/
      
        LidarDrv.bufferState_A = IS_EMPTY;
    }
    else if ((enum_buffer_state*)arg == &(LidarDrv.bufferState_B))
    {        
        LidarDrv.bufferState_B = IS_READING;
      
        /*�û�ϣ����һȦ���ݽ��еĲ��������������*/
      
        printf ("LidarDrv.validNodePos_B = %d\nLidarDrv.ready_time_ms_B = %d\n", LidarDrv.validNodePos_B, LidarDrv.ready_time_ms_B);
        HAL_Delay (80);    //����ʱ����ģ�⼫�޸��ɴ���ʱ�䣬���ʱ��ɾ��
        /*for (unsigned int j = 0; j < LidarDrv.validNodePos_B; j++)
        {
            if (LidarDrv.nodeBuffer_B[j].quality > 0)
            {
                printf ("A = %.2f  D = %d\n", LidarDrv.nodeBuffer_B[j].angle_z_q14 * 90.f / (1 << 14), LidarDrv.nodeBuffer_B[j].dist_mm_q2 / (1 << 2));
            }
        }*/
      
        /*�û�ϣ����һȦ���ݽ��еĲ��������������*/
      
        LidarDrv.bufferState_B = IS_EMPTY;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init(DEFAULT_MOTOR_PWM);
  /* USER CODE BEGIN 2 */
  
  huart1.Init.BaudRate = lidar_baudrate;    //����UART1�Ĳ�����
  HAL_UART_Init(&huart1);
    
  huart1.Init.BaudRate = 256000;            //����UART2�Ĳ�����
  HAL_UART_Init(&huart2);
    
  huart1.Init.BaudRate = 256000;            //����UART6�Ĳ�����
  HAL_UART_Init(&huart6);
  
  uart1_rx_buffer_init ();   //UART1 Rx DMA buffer��ʼ��
  
  Ring_Buffer_Init ();       //���ζ��г�ʼ��
  
  LidarDrvInit ();           //�״�������ʼ����
  
  LidarDrv.stopScan ();     //ͨ���ȱʡ��ֹͣһ��ɨ��
  
  LidarDrv.stopMotor ();    //ͨ���ȱʡ��ֹͣһ�µ��
  
  if (LidarDrv.getDeviceInfo (500) == TRUE)    //��ȡ�״���Ϣ
  {
      printf ("Model : %x\nFirmware : %d.%d\nHardware : %02d\nSN : ", LidarDrv.lidar_info.model, LidarDrv.lidar_info.firmware_version_major, LidarDrv.lidar_info.firmware_version_minor, LidarDrv.lidar_info.hardware_version);
    
      for (unsigned char i = 0; i < 16; i++)
      {
          printf ("%02X", LidarDrv.lidar_info.serialnum[i]);
      }
      
      printf ("\n");
      
      LidarDrv.isConnected = TRUE;               //�������������ȡ�״���Ϣ����ô�жϺ��״��Ѿ�����
      
      if (LidarDrv.getHealth (500) == TRUE)      //��ȡ�״｡����Ϣ
      {
          printf ("Health Info : %x\nError Code : %d\n", LidarDrv.health_info.status, LidarDrv.health_info.error_code);
      }
      
      if (LidarDrv.checkMotorCtrlSupport (500) == FALSE)    //��������豸�Ƿ�֧�ֵ��ת�ٿ��ƣ����RPLidar���ӵ�RPLidar A2��A3��S1��USBת�Ӱ壬�˺����᷵��TRUE����ʱ���ӵ���STM32F401RE�����壬�������岻����Ӧ�˴β�ѯ���ǿ�������֧��PWM���Ƶ��ת�ٵģ�
      {  
          LidarDrv.isSupportingMotorCtrl = TRUE;

          if (LidarDrv.isSupportingMotorCtrl == TRUE)
          {
            printf ("SupportMotorCtrl : True\n");
          }
          else
          {
            printf ("SupportMotorCtrl : False\n");    
          }
      }
      
      LidarDrv.getAllSupportedScanModes (2000);             //��ȡ����֧�ֵ�ɨ��ģʽ�Լ���ɨ��ģʽ����ϸ��Ϣ��

      printf ("ScanModeCount : %d\n",LidarDrv.ScanModeCount);
      
      printf ("TypicalScanModeID : %d\n",LidarDrv.TypicalScanModeID);
    
      unsigned short temp;
                
      if (LidarDrv.ScanModeCount > MAX_SCAN_MODE_COUNT)    //������ʾMAX_SCAN_MODE_COUNT��ɨ��ģʽ����ϸ��Ϣ��ʣ�µ��޷���ʾ��
          temp = MAX_SCAN_MODE_COUNT;
      else
          temp = LidarDrv.ScanModeCount;
                  
      for (unsigned short j = 0; j < temp; j++)
      {
          printf ("ID  %d  :  %s  %d Points/Sec  %dm  0x%02X\n\n", LidarDrv.ScanMode[j].id, LidarDrv.ScanMode[j].scan_mode, (int)(1000000/LidarDrv.ScanMode[j].us_per_sample), (int)(LidarDrv.ScanMode[j].max_distance), LidarDrv.ScanMode[j].ans_type);
      }
  }
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      unsigned char scan_mode_select = 1;    //�û�ѡ���ɨ��ģʽID
      
      LidarDrv.startMotor ();   //���������
      
      HAL_Delay (100);
      
      if (LidarDrv.startScan (FALSE, scan_mode_select, 500) == TRUE)  //����ɨ�裬��2������Ϊɨ��ģʽID���״�֧�ֵ�ɨ��ģʽ�����Լ���ϸ��Ϣ��μ�����ġ�ɨ��ģʽ��ѯ�Ľ����
      {          
          curr_wait_size = UART1_RX_BUFFER_SIZE;    next_wait_size = curr_wait_size;
          
          curr_using_buffer = add_uart1_rx_task(curr_wait_size);
          
          if (curr_using_buffer != NULL) 
          {
              prev_wait_size = curr_wait_size;
            
              LidarDrv.current_cache_proc (NULL);    //�����״����������������״�������������������ѭ�� !!!  
          }
      }
      /* USER CODE BEGIN 3 */
      
      /* USER CODE END 3 */
  }
  /* USER CODE END WHILE */
  LidarDrv.stopScan ();    //ֹͣɨ��
  
  LidarDrv.stopMotor ();   //ֹͣ���
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(unsigned short pwm)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pwm;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;                  //�趨UART������
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  //status = HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer_A, UART1_RX_BUFFER_SIZE);
  //uart1_rx_buffer_A_state = IS_WRITING;
  
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;                  //�趨UART������
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 256000;                  //�趨UART������
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  
  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
