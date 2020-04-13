/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *
  *   Author of non-automatic generated code: papamidas (DM1CR)
  *   Copyright 2020 DM1CR
  *   The non-automatic generated code can be used under the GNU public license
  *
  */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DACMAX 4095
#define DACMID 2048
#define HALFBUFFERSIZE 512
#define FULLBUFFERSIZE 1024
#define SINTABLEN 65536
#define PHASEINCRESETVAL 85899346


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
__attribute__((section(".dma_buffer"))) uint32_t dacbuf[FULLBUFFERSIZE];

volatile uint32_t* dacbufptr = dacbuf;
volatile uint32_t phase1;
volatile uint32_t phase2;
volatile uint16_t* phase1_16ptr;
volatile uint16_t* phase2_16ptr;
uint32_t phaseinc1;
uint32_t phaseinc2;
uint16_t sintab[SINTABLEN];
uint16_t resetphase = 0;
uint16_t tracedummy = 0;

/* Pofiling/Benchmarking with ARM cycle counter
 */
volatile uint32_t cycleCounter = 0;
// addresses of registers
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef HAL_DUALDAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment);
void ITM_enable(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* calculation of the output frequency of the DDS:
 * FOUT = phaseinc * DACCLOCK / 2**PHASEBITS
 *
 * DACCLOCK: 	TIM6 is used for generating the DAC clock
 * 				TIM6 is driven by the APB1 clock, typ. 200 MHz
 * 				TIM6 prescaler and counter period determine the DAC clock
 * 				Example: 	APB1 clock = 200 MHz
 * 							prescaler = 0 (=clk division by 1)
 * 							counter period = 39
 * 				-> DACCLOCK = 200 MHz / (39+1) = 5 MHz
 *
 * phaseinc:    phaseinc = FOUT/DACCLOCK * 2**phasebits
 * 				Example:	FOUT = 100 kHz
 * 							DACCLOCK = 5 MHz
 * 							phasebits = 32
 * 				-> phaseinc = 100 kHz/5 MHz * ‭4294967296‬ = ‭‭85899346
 *
 * 	calculation of the frequency resolution of the DDS:
 * 	FRES = DACCLOCK / 2**PHASEBITS
 *
 * 				Example:	DACCLOCK = 5 MHz
 * 							phasebits = 32
 * 				-> FRES = 5 MHz / ‭4294967296 = 1,16 MilliHertz
 *
 */
void dds()
{
	for (int i=0; i<HALFBUFFERSIZE; i++){
		phase1 += phaseinc1;              //increment phase accumulator1 by phase increment value
		phase2 += phaseinc2;              //increment phase accumulator2 by phase increment value
		//HAL_GPIO_WritePin(PROFILE2_GPIO_Port, PROFILE2_Pin, GPIO_PIN_SET);
		*dacbufptr++ = sintab[*((uint16_t*)&phase1 + 1)] +
				       ((uint32_t)sintab[*((uint16_t*)&phase2 + 1)] << 16);
		//HAL_GPIO_WritePin(PROFILE2_GPIO_Port, PROFILE2_Pin, GPIO_PIN_RESET);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	HAL_GPIO_WritePin(PROFILE0_GPIO_Port, PROFILE0_Pin, GPIO_PIN_SET);
	cycleCounter = *DWT_CYCCNT;  // store current CPU cycle counter value
    dacbufptr = &dacbuf[HALFBUFFERSIZE];
    dds();
	tracedummy = *DWT_CYCCNT - cycleCounter;  // calculate elapsed CPU cycles
	HAL_GPIO_WritePin(PROFILE0_GPIO_Port, PROFILE0_Pin, GPIO_PIN_RESET);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    dacbufptr = dacbuf;
    dds();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  for(int i=0; i<SINTABLEN; i++)
  {
	  sintab[i] = (DACMID - 1) * 0.9 * sin(i * 2.0 * M_PI / SINTABLEN) + DACMID;
  }

  phaseinc1 = PHASEINCRESETVAL;
  printf("phaseinc1 = ");
  printf("%lu\n", phaseinc1);
  phaseinc2 = PHASEINCRESETVAL;
  printf("phaseinc2 = ");
  printf("%lu\n", phaseinc2);
  //phaseinc2 = 55;

  /* Benchmarking/profiling:
   *
  */// enable the use DWT
  *DEMCR = *DEMCR | 0x01000000;
  // Reset cycle counter
  *DWT_CYCCNT = 0;
  // enable cycle counter
  *DWT_CONTROL = *DWT_CONTROL | 1 ;

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
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_DUALDAC_Start_DMA(&hdac1, dacbuf, FULLBUFFERSIZE, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim6);
  ITM_enable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      if (resetphase > 0) {
    	  __disable_irq();
    	  phase1 = 0;
    	  phase2 = 0;
    	  phaseinc1 = PHASEINCRESETVAL;
    	  phaseinc2 = PHASEINCRESETVAL;
    	  resetphase = 0;
    	  __enable_irq();
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_1);
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
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 39;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|PROFILE0_Pin|PROFILE2_Pin 
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PROFILE3_GPIO_Port, PROFILE3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin PROFILE0_Pin PROFILE2_Pin 
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|PROFILE0_Pin|PROFILE2_Pin 
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PROFILE3_Pin */
  GPIO_InitStruct.Pin = PROFILE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PROFILE3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Enables DUAL_DAC mode and starts conversion of both channels.
  *         ****** modified version of HAL_DAC_Start_DMA from DM1CR, Apr 5, 2020 *******
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  pData The destination peripheral Buffer address.
  * @param  Length The length of data to be transferred from memory to DAC peripheral
  * @param  Alignment Specifies the data alignment for DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DUALDAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment)
{
  HAL_StatusTypeDef status;
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(Channel));
  assert_param(IS_DAC_ALIGN(Alignment));

  /* Process locked */
  __HAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = HAL_DAC_STATE_BUSY;

  /* Set the DMA transfer complete callback for channel1 */
  hdac->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltCh1;

  /* Set the DMA half transfer complete callback for channel1 */
  hdac->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh1;

  /* Set the DMA error callback for channel1 */
  hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;

  /* Enable the selected DAC channel1 DMA request */
  SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN1);

  switch (Alignment)  {
      case DAC_ALIGN_12B_R:
                             /* Get dual mode DHR12RD address */
    	                     tmpreg = (uint32_t)&hdac->Instance->DHR12RD;
    	                     break;
      case DAC_ALIGN_12B_L:
                             /* Get dual mode DHR12LD address */
                             tmpreg = (uint32_t)&hdac->Instance->DHR12LD;
                             break;
      case DAC_ALIGN_8B_R:
                             /* Get dual mode DHR8RD address */
                             tmpreg = (uint32_t)&hdac->Instance->DHR8RD;
                             break;
      default:
                             break;
  }


  /* Enable the DMA Stream */
  /* Enable the DAC DMA underrun interrupt */
  __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

  /* Enable the DMA Stream */
  status = HAL_DMA_Start_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);

  /* Process Unlocked */
  __HAL_UNLOCK(hdac);

  if (status == HAL_OK)
  {
    /* Enable the Peripheral */
    __HAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
    __HAL_DAC_ENABLE(hdac, DAC_CHANNEL_2);
  }
  else
  {
    hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
  }

  /* Return function status */
  return status;
}

void ITM_enable(void)
{
#define SWO_BASE (0x5C003000UL)
#define SWTF_BASE (0x5C004000UL)
 __IO GPIO_TypeDef *GPIOBRegs = (GPIO_TypeDef *)GPIOB;

 uint32_t SWOSpeed = 2000000; /* [Hz] we have 2 Mbps SWO speed in ST-Link SWV viewer
 if we select 400000000 Hz core clock */
 uint32_t SWOPrescaler = (SystemCoreClock / SWOSpeed) - 1; /* divider value */

 //enable debug clocks
 DBGMCU->CR = 0x00700000; //enable debug clocks

 //UNLOCK FUNNEL
 //SWTF->LAR unlock
 *((__IO uint32_t *)(SWTF_BASE + 0xFB0)) = 0xC5ACCE55; //unlock SWTF

 //SWO->LAR unlock
 *((uint32_t *)(SWO_BASE + 0xFB0)) = 0xC5ACCE55; //unlock SWO

 //SWO divider setting
 //This divider value (0x000000C7) corresponds to 400Mhz core clock
 //SWO->CODR = PRESCALER[12:0]
 *((__IO uint32_t *)(SWO_BASE + 0x010)) = SWOPrescaler; //clock divider

 //SWO set the protocol
 //SWO->SPPR = PPROT[1:0] = NRZ
 *((__IO uint32_t *)(SWO_BASE + 0x0F0)) = 0x00000002; //set to NRZ

 //Enable ITM input of SWO trace funnel, slave 0
 //SWTF->CTRL bit 0 ENSO = Enable
 *((__IO uint32_t *)(SWTF_BASE + 0x000)) |= 0x00000001; //enable

 //RCC_AHB4ENR enable GPIOB clock - maybe done already
 RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;

 //Configure GPIOB_MODER pin 3 as AF
 GPIOBRegs->MODER &= ~GPIO_MODER_MODE3; //clear MODER3 bits
 GPIOBRegs->MODER |=
 GPIO_MODE_AF_PP << GPIO_OSPEEDR_OSPEED3_Pos; //set MODER3 PIN3 bits as AF

 //Configure GPIOB_OSPEEDR pin 3 Speed
 GPIOBRegs->OSPEEDR &=
 ~GPIO_OSPEEDR_OSPEED3; //clear OSPEEDR3 bits
 GPIOBRegs->OSPEEDR |=
 GPIO_SPEED_FREQ_HIGH << GPIO_OSPEEDR_OSPEED3_Pos; //set OSPEEDR3 PIN3 bits as High Speed

 //Force AF0 for GPIOB_AFRL, AFR[0] for pin 3
 GPIOBRegs->AFR[0] &= ~GPIO_AFRL_AFRL3; //clear AFR2 PIN3 = 0 for AF0
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
