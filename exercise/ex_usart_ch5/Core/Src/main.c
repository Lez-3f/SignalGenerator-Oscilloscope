/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD*/

//5.1-2: 1, 5.3.1:3, 5.1.4:4
#define RXBUFFERSIZE 3 //接收缓冲区的长度
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[RXBUFFERSIZE];

// 5.2
uint8_t CommOkMessage[] = "Everything is OK\r\n";
uint8_t CommErrMessage[] = "Received Error Data\r\n";
uint8_t CommFlag = 0;
uint8_t RxBuffer[RXBUFFERSIZE] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void sort(uint32_t* a, uint32_t length, uint32_t* b);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, RXBUFFERSIZE);

  // 5.4
//  printf("Please Enter Data:\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 5.1.1
//	  if (RxBuffer[0]==0x10){
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	  }
//	  else if (RxBuffer[0]==0x20){
//		  HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
//	  }
//	  else{
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
//	  }

	  // 5.1.2 控制LD的闪烁次数
//	  uint8_t num = (RxBuffer[0] < 20) ? RxBuffer[0] : 20;
//	  for (uint8_t i = 0; i < num * 2; ++i)
//	  {
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		  HAL_Delay(500);
//	  }
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(2000);

	  // 5.2.1
//	  if ((RxBuffer[0] == 0x10)&&( CommFlag == 1)) {
//		  CommFlag = 0;
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//		  HAL_UART_Transmit(&huart2,CommOkMessage,19,1000);
//	  }
//	  else if ((RxBuffer[0] != 0x10)&&( CommFlag == 1)) {
//		  CommFlag = 0;
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//		  HAL_UART_Transmit(&huart2,CommErrMessage,22,1000);
//	  }
	  // 5.2.2
//	  uint8_t num = 0;
//	  uint8_t i = 0;
//	  if (RxBuffer[0] < 10 && CommFlag == 1){
//		  CommFlag = 0;
//		  num = RxBuffer[0];
//		  i = 0;
//		  for (i = 1; i <= num; ++i)
//		  {
//			  uint8_t flash_msg[16];
//			  sprintf((char *)flash_msg, "flash %d times\r\n", i);
//			  HAL_UART_Transmit(&huart2, flash_msg, 16, 1000);
//			  HAL_Delay(20);
//			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//			  HAL_Delay(500);
//			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//			  HAL_Delay(500);
//		  }
//		  uint8_t flash_end_msg[] = "flash over\r\n";
//		  HAL_UART_Transmit(&huart2, flash_end_msg, 13, 1000);
//		  HAL_Delay(2000);
//	  }

	  // 5.3.1
//	  if ((RxBuffer[0] == 0x53)&&(RxBuffer[2] == 0x45)&&(CommFlag == 1))
//	  {
//		  CommFlag = 0;
//		  if (RxBuffer[1] == 0x10)
//		  {
//			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//			  HAL_UART_Transmit_IT(&huart2, CommOkMessage, 19);
//		  }
//		  else
//		  {
//			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//			  HAL_UART_Transmit_IT(&huart2, CommErrMessage, 22);
//		  }
//	  }

	  // 5.3.2
//	  uint16_t f_delay;
//	  uint8_t f_times;
//	  uint8_t i;
//	  if ((RxBuffer[0] == 0x53)&&(RxBuffer[3] == 0x45)&&(CommFlag == 1))
//	  {
//		  f_delay = (uint16_t)RxBuffer[1] * 10;
//		  f_times = RxBuffer[2];
//		  CommFlag = 0;
//
//		  i = 0;
//		  for (i = 1; i <= f_times; ++i)
//		  {
//		  			  uint8_t flash_msg[16];
//		  			  sprintf((char *)flash_msg, "flash %d times\r\n", i);
//		  			  HAL_UART_Transmit(&huart2, flash_msg, 16, 1000);
//		  			  HAL_Delay(20);
//		  			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//		  			  HAL_Delay(f_delay);
//		  			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//		  			  HAL_Delay(f_delay);
//		  }
//		  		  uint8_t flash_end_msg[] = "flash over\r\n";
//		  		  HAL_UART_Transmit(&huart2, flash_end_msg, 13, 1000);
//		  		  HAL_Delay(2000);
//	  	}

	  // 5.5
	  printf("calculate the ranks of 20 students's scores, start:\r\n");

	  uint32_t scores[20];
	  uint32_t scores_copy[20];
	  uint32_t sort_idx[20];
	  for (uint8_t i = 0;i < 20; ++i){
		  sort_idx[i] = i;
	  }
	  for (uint8_t i = 0;i < 20; ++i){
		  printf("please enter %d th students's score:\r\n", i);
		  while(1){
		   if (CommFlag == 1){

			  uint32_t score = 0;
			  CommFlag = 0;
			  score = (RxBuffer[0]-48)*100 + (RxBuffer[1]-48)*10 + RxBuffer[2]-48;
			  if (score <= 100){
				  scores[i] = score;
			  	  printf("receive score: %d of %d th student\r\n", score, i);
			  	  break;
			  }
			  else{
				  printf("invalud score, please enter again\r\n");
			  }
		   }
		  }
	  }
	  HAL_Delay(200);
	  printf("all scores:\r\n");
	  for (uint8_t i = 0;i < 20; ++i){
		  printf("%d: %d\r\n", i, scores[i]);
	  }

	  for(uint8_t i =0; i < 20; ++i){
		  scores_copy[i] = scores[i];
	  }
	  sort(scores_copy, 20, sort_idx);

	  printf("after sorted:\r\n");
	  for (uint8_t i = 0;i < 20; ++i){
		  printf("%d: stu %d of score %d\r\n",i, sort_idx[i], scores[sort_idx[i]]);
	  }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZ_Pin|LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZ_Pin LED_Pin */
  GPIO_InitStruct.Pin = BUZ_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	CommFlag = 1;
	HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuffer, RXBUFFERSIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
	for(uint8_t i = 0; i < RXBUFFERSIZE; i++)
	{
		printf("RxBuffer[%d] = 0x%02x\r\n", i, RxBuffer[i]);
	}
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void sort(uint32_t* a, uint32_t length, uint32_t* b)
{
	uint32_t i,j, t1, t;
    for(j=0; j<length; j++)
        for(i=0; i<length-1-j; i++)
            if(a[i]<a[i+1])
            {
                t=a[i];
                a[i]=a[i+1];
                a[i+1]=t;


                t1=b[i];
                b[i]=b[i+1];
                b[i+1]=t1;
            }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
