
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t ReceiveBuff[BUFFERSIZE];						//发送缓冲区
uint8_t TransmitBuff[BUFFERSIZE];						//接收缓冲区

uint8_t recv_end_flag = 0;	//接收完成中断，接收到字符长度
uint8_t Rx_len = 0;				  // 接收到的字符长度

uint16_t g_Error_Count = 0;  // 统计接收到的错误帧
int32_t g_last_recv_number = -1;

uint8_t get_flag = 0;

// 检查接收到的数据是否正确、
void Check_Buff(uint8_t* RecvBuff);

// 根据接收到的数据，填充发送的数据
void Fill_Buffer(uint8_t* RecvBuff,uint8_t* TransBuff);

// 32位的数据转换为uint8的数组
void uint32_to_uint8(uint32_t number,uint8_t* data);

// 将uint8的数组转换成32位的数据
uint32_t uint8_to_uint32(uint8_t data[]);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i;
	uint32_t tick = 0;
	uint8_t len = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	printf("Hell\r\n");
	

	for(i = 0;i<10;i++)
	{
			TransmitBuff[i] = 0x0;
	}
		
	// 转为发送状态
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)TransmitBuff,10);
	HAL_Delay(5);
	// 转为接收状态
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	
	tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(recv_end_flag==1)
		{
			tick = HAL_GetTick();
			
			printf("Recv Data:");
			for (i = 0;i<Rx_len;i++)
			{
				printf("%x ",ReceiveBuff[i]);

			}
			printf(",\terror_count = %d\r\n",g_Error_Count);
			Rx_len=0;
			recv_end_flag=0;
			
			
			Check_Buff(ReceiveBuff);
			// 根据接收的数据，填充发送的数据
			Fill_Buffer(ReceiveBuff,TransmitBuff);
			
  
			// 关闭串口
			huart3.Instance->CR1&= (~(uint32_t)0x0004);
			// 转为发送状态
			//HAL_Delay(5);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_Delay(100);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)TransmitBuff,6);
			//while(huart2.hdmatx->Instance->CNDTR);
			// 转为接收状态
			
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			
						// 打开串口
			huart3.Instance->CR1|= (uint32_t)0x0004;
			
		}
		else if((HAL_GetTick()-tick)>5000)
		{
//			len=Rx_len;            //得到此次接收到的数据长度
			tick = HAL_GetTick();
			for(i = 0;i<len;i++)
			{
				TransmitBuff[i] = i;
			}
//			
			// 关闭串口
			huart3.Instance->CR1&= (~(uint32_t)0x0004);
			// 转为发送状态
			//HAL_Delay(5);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_Delay(10);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)TransmitBuff,6);
			//while(huart2.hdmatx->Instance->CNDTR);
			// 转为接收状态
			
			HAL_Delay(5);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			
			// 打开串口
			huart3.Instance->CR1|= (uint32_t)0x0004;
		
		}
		
		HAL_UART_Receive_DMA(&huart3,(uint8_t*)ReceiveBuff,BUFFERSIZE); 
		
		// 插拔之后可能会出现LBD、ORE标志位置位，需要及时检测，进行复位。
		// 在ORE标志位置位之后，由于缓冲区溢出，RXNE标志位也会跟着置位，所以也要及时清空，开不开中断无所谓。
		// 在串口初始化时打开了ERR中断，遇到ORE标志位置位会进入ErrCallBack函数，为什么不在中断函数里进行置位呢，
		// 因为可能在接收数据的函数里把Err中断给关掉了，所以只能在主函数里进行清空。
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_LBD)==SET)
		{
			__HAL_UART_CLEAR_FLAG(&huart3,UART_FLAG_LBD);
		}
//		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)==SET)
//		{
//			
//		}
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE) != RESET) 
    {
			 //HAL_UART_Receive_IT(huart,(uint8_t *)&i,1);
			 __HAL_UART_CLEAR_FLAG(&huart3,UART_FLAG_RXNE);
       __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}
void Check_Buff(uint8_t* RecvBuff)
{
	uint32_t number = 0;
	number = uint8_to_uint32(RecvBuff);
	if(number!=(g_last_recv_number+2))
	{
		g_Error_Count++;
	}
	else
	{
		g_last_recv_number = number;
	}
}


void Fill_Buffer(uint8_t* RecvBuff,uint8_t* TransBuff)
{
		uint32_t number = 0;
		number = uint8_to_uint32(RecvBuff);
		number++;
		uint32_to_uint8(number,TransBuff);
	
		TransBuff[4] = 0xa;
		TransBuff[5] = 0xa;
}


void CopyBuffer(uint8_t* source,uint8_t* dest,uint8_t len)
{
		int i = 0; 
		for(i = 0; i<len;i++)
		{
				dest[i] = source[i];
		}

}

void uint32_to_uint8(uint32_t number,uint8_t data[])
{
	int i = 0;
	for(i = 0; i < 4; i++)
	{
		data[i] = number>>(8*(3-i));
	
	}
	
//    data[0] = number>>24;
//    data[1] = number>>16;
//    data[2] = number>>8;
//    data[3] = number;
}
uint32_t uint8_to_uint32(uint8_t data[]){
    uint32_t number = 0;
    number = number | data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
    return number;
}	
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
