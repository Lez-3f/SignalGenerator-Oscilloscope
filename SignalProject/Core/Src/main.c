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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define ADC_CONVERTED_DATA_BUFFER_SIZE (uint16_t) 60

#define ADC_CONVERTED_DATA_BUFFER_SIZE (uint16_t) 2000

//#define DAC_BUFFER_SIZE (uint16_t) 50
#define DAC_BUFFER_SIZE (uint16_t) 200

#define SINE_MODE 0
#define SQUARE_MODE 1
#define TRIANGLE_MODE 2
#define MODE_NUM 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint16_t SineWaveData[DAC_BUFFER_SIZE] = {2047,2304,2557,2801,3034,3251,3449,3625,3776,3900,3994,4058,4090,4090,
//		4058,3994,3900,3776,3625,3449,3251,3034,2801,2557,2304,2048,1791,1538,1294,
//		1061,844,646,470,319,195,101,37,5,5,37,101,195,319,470,646,844,1061,1294,1538, 1791};


//uint16_t SineWaveData[DAC_BUFFER_SIZE] = {
//2047,2112,2176,2240,2304,2368,2431,2494,2557,2619,2680,2741,2801,2860,2919,2977,3034,3090,3144,3198,
//3251,3302,3352,3401,3449,3495,3540,3583,3625,3665,3704,3741,3776,3809,3841,3871,3900,3926,3951,3973,
//3994,4013,4030,4045,4058,4069,4078,4085,4090,4093,4094,4093,4090,4085,4078,4069,4058,4045,4030,4013,
//3994,3973,3951,3926,3900,3871,3841,3809,3776,3741,3704,3665,3625,3583,3540,3495,3449,3401,3352,3302,
//3251,3198,3144,3090,3034,2977,2919,2860,2801,2741,2680,2619,2557,2494,2431,2368,2304,2240,2176,2112,
//2048,1983,1919,1855,1791,1727,1664,1601,1538,1476,1415,1354,1294,1235,1176,1118,1061,1005,951,897,844,
//793,743,694,646,600,555,512,470,430,391,354,319,286,254,224,195,169,144,122,101,82,65,50,37,26,17,10,
//5,2,0,2,5,10,17,26,37,50,65,82,101,122,144,169,195,224,254,286,319,354,391,430,470,512,555,600,646,694,
//743,793,844,897,951,1005,1061,1118,1176,1235,1294,1354,1415,1476,1538,1601,1664,1727,1791,1855,1919,1983
//};

//uint16_t NoneData[DAC_BUFFER_SIZE] = {0};

uint16_t Sine1WaveData[DAC_BUFFER_SIZE] = {
		2047,2112,2176,2240,2304,2368,2431,2494,2557,2619,2680,2741,2801,2860,2919,2977,3034,3090,3144,3198,3251,3302,3352,3401,3449,3495,3540,3583,3625,3665,3704,3741,3776,3809,3841,3871,3900,3926,3951,3973,3994,4013,4030,4045,4058,4069,4078,4085,4090,4093,4094,4093,4090,4085,4078,4069,4058,4045,4030,4013,3994,3973,3951,3926,3900,3871,3841,3809,3776,3741,3704,3665,3625,3583,3540,3495,3449,3401,3352,3302,3251,3198,3144,3090,3034,2977,2919,2860,2801,2741,2680,2619,2557,2494,2431,2368,2304,2240,2176,2112,2048,1983,1919,1855,1791,1727,1664,1601,1538,1476,1415,1354,1294,1235,1176,1118,1061,1005,951,897,844,793,743,694,646,600,555,512,470,430,391,354,319,286,254,224,195,169,144,122,101,82,65,50,37,26,17,10,5,2,0,2,5,10,17,26,37,50,65,82,101,122,144,169,195,224,254,286,319,354,391,430,470,512,555,600,646,694,743,793,844,897,951,1005,1061,1118,1176,1235,1294,1354,1415,1476,1538,1601,1664,1727,1791,1855,1919,1983
};

uint16_t SquareWaveData[DAC_BUFFER_SIZE] = {
		4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint16_t TriangleWaveData[DAC_BUFFER_SIZE] = {
		2047,2089,2131,2172,2214,2256,2298,2339,2381,2423,2465,2507,2548,2590,2632,2674,2715,2757,2799,2841,2883,2924,2966,3008,3050,3091,3133,3175,3217,3258,3300,3342,3384,3426,3467,3509,3551,3593,3634,3676,3718,3760,3802,3843,3885,3927,3969,4010,4052,4094,4094,4052,4010,3969,3927,3885,3843,3802,3760,3718,3676,3634,3593,3551,3509,3467,3426,3384,3342,3300,3258,3217,3175,3133,3091,3050,3008,2966,2924,2883,2841,2799,2757,2715,2674,2632,2590,2548,2507,2465,2423,2381,2339,2298,2256,2214,2172,2131,2089,2047,2047,2005,1963,1922,1880,1838,1796,1755,1713,1671,1629,1587,1546,1504,1462,1420,1379,1337,1295,1253,1211,1170,1128,1086,1044,1003,961,919,877,836,794,752,710,668,627,585,543,501,460,418,376,334,292,251,209,167,125,84,42,0,0,42,84,125,167,209,251,292,334,376,418,460,501,543,585,627,668,710,752,794,836,877,919,961,1003,1044,1086,1128,1170,1211,1253,1295,1337,1379,1420,1462,1504,1546,1587,1629,1671,1713,1755,1796,1838,1880,1922,1963,2005,2047
};

//uint16_t (*waveDatas)[3] = {Sine1WaveData, SquareWaveData, TriangleWaveData};

//uint16_t HarmonicSineWaveData[DAC_BUFFER_SIZE] = {
//		2047,2147,2246,2344,2441,2536,2629,2718,2805,2888,2968,3043,3113,3179,3240,3296,3347,3393,3433,
//		3467,3497,3521,3540,3554,3563,3567,3567,3564,3556,3545,3530,3514,3494,3473,3450,3427,3402,3377,
//		3352,3328,3304,3282,3261,3241,3224,3209,3196,3186,3179,3175,3173,3175,3179,3186,3196,3209,3224,
//		3241,3261,3282,3304,3328,3352,3377,3402,3427,3450,3473,3494,3514,3530,3545,3556,3564,3567,3567,
//		3563,3554,3540,3521,3497,3467,3433,3393,3347,3296,3240,3179,3113,3043,2968,2888,2805,2718,2629,
//		2536,2441,2344,2246,2147,2048,1948,1849,1751,1654,1559,1466,1377,1290,1207,1127,1052,982,916,855,
//		799,748,702,662,628,598,574,555,541,532,528,528,531,539,550,565,581,601,622,645,668,693,718,743,767,
//		791,813,834,854,871,886,899,909,916,920,922,920,916,909,899,886,871,854,834,813,791,767,743,718,693,668,
//		645,622,601,581,565,550,539,531,528,528,532,541,555,574,598,628,662,702,748,799,855,916,982,1052,1127,
//		1207,1290,1377,1466,1559,1654,1751,1849,1948
//};



uint16_t ADC1ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
uint16_t FrameHeader = 0x5353;
uint16_t FrameTerm = 0x4545;

uint16_t mode = SINE_MODE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)Sine1WaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&ADC1ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	 switch(mode){
//	 case SINE_MODE:{
//
//		 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)SineWaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
//		 break;
//	 }
//	 case SQUARE_MODE:{
//		 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)SquareWaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
//		 break;
//	 }
//	 case TRIANGLE_MODE:{
//		 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)TriangleWaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
//		 break;
//	 }
//	 }
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV10;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 10000000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&FrameHeader,2, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t *)&ADC1ConvertedData,
	ADC_CONVERTED_DATA_BUFFER_SIZE*2, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t *)&FrameTerm,2, 0xFFFF);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(10);
	if (GPIO_Pin == KEY_Pin){
		mode = (mode + 1) % MODE_NUM;
		HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);

		switch(mode){
			 case SINE_MODE:{
				 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)Sine1WaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
				 break;
			 }
			 case SQUARE_MODE:{
				 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)SquareWaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
				 break;
			 }
			 case TRIANGLE_MODE:{
				 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t *)TriangleWaveData,DAC_BUFFER_SIZE,DAC_ALIGN_12B_R);
				 break;
			 }
		}
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
