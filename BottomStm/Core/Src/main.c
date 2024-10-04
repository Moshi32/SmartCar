/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "i2c-lcd.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void otoparkP(void);
void otoparkL(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


 void delay(uint16_t time)
{

	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < time);
}

uint8_t compare = 0;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 99;

#define TRIG_PIN GPIO_PIN_1
#define TRIG_PORT GPIOC

uint8_t compare2 = 0;
uint32_t IC_Val21 = 0;
uint32_t IC_Val22 = 0;
uint32_t Difference2 = 0;
uint8_t Is_First_Captured2 = 0;  // is the first value captured ?
uint8_t Distance2  = 0;

#define TRIG_PIN2 GPIO_PIN_8
#define TRIG_PORT2 GPIOA


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
		{
			if (Is_First_Captured2==0) // if the first value is not captured
			{
				IC_Val21 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
				Is_First_Captured2 = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured2==1)   // if the first is already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				if (IC_Val22 > IC_Val21)
				{
					Difference2 = IC_Val22-IC_Val21;
				}

				else if (IC_Val21 > IC_Val22)
				{
					Difference2 = (0xffff - IC_Val21) + IC_Val22;
				}

				Distance2 = Difference2 * .034/2;
				Is_First_Captured2 = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
			}
		}
}


void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(2);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	HAL_GPIO_WritePin(TRIG_PORT2, TRIG_PIN2, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
}

	uint8_t rxTemBuf[10];
	uint8_t rxBuf[100];
	uint8_t rxIndex = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	if(huart == &huart1)
	{
		rxBuf[rxIndex] = rxTemBuf[0];
		HAL_UART_Receive_IT(&huart1, rxTemBuf,1);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

  TIM2->CCR1 = compare;
  TIM2->CCR2 = compare;
  TIM3->CCR1 = compare;
  TIM3->CCR2 = compare;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart1, rxTemBuf,1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HCSR04_Read();

	  if(rxBuf[rxIndex] == 'F') // forward
	  {

		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		  compare = 80;
		  TIM2->CCR1 = compare + 1; // sağ teker
		  TIM2->CCR2 = compare ; // sol teker
		  TIM3->CCR1 = 0; // sol teker ters
		  TIM3->CCR2 = 0; // sağ teker ters
		  rxIndex++;

      }

	  else if(rxBuf[rxIndex] == 'B') // reverse
	  {

		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		  compare = 0;
		  TIM2->CCR1 = compare;
		  TIM2->CCR2 = compare;
		  TIM3->CCR1 = compare + 80;
		  TIM3->CCR2 = compare + 80 + 1;
		  rxIndex++;

	  }

	  else if(rxBuf[rxIndex] == 'V') // Left
	  {
	  compare = 80;
	  TIM2->CCR1 = compare; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'R') // Right
	  {
	  compare = 80;
	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = compare; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'G') // forward Left
	  {
	  compare = 80;
	  TIM2->CCR1 = compare + 4; // sağ teker
	  TIM2->CCR2 = compare - 4; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'I') // forward Right
	  {
	  compare = 80;
	  Distance = 5;
	  TIM2->CCR1 = compare - 4; // sağ teker
	  TIM2->CCR2 = compare + 4; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'H') // reverse Left
	  {
	  compare = 80;
	  Distance = 5;
	  TIM3->CCR1 = compare - 4; // sağ teker
	  TIM3->CCR2 = compare + 4; // sol teker
	  TIM2->CCR1 = 0; // sol teker ters
	  TIM2->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'J') // reverse Right
	  {
	  compare = 80;
	  Distance = 5;
	  TIM3->CCR1 = compare + 4; // sağ teker
	  TIM3->CCR2 = compare - 4; // sol teker
	  TIM2->CCR1 = 0; // sol teker ters
	  TIM2->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'W') // Parallel park
	  {
		  otoparkP();
	  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'U') // L park
	  {
		  otoparkL();
	  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'S') // forward stop
	  {
	  compare = 0;
	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters
  	  rxIndex++;
	  }

	  rxBuf[rxIndex] = 0;





	  /*if(Distance < 10) {
		  compare = 0;
		  TIM2->CCR1 = 0; // sağ teker
		  TIM2->CCR2 = 0; // sol teker
		  TIM3->CCR1 = 0; // sol teker ters
		  TIM3->CCR2 = 0; // sağ teker ters
	  }*/

	  /*else if(rxBuf[rxIndex] == 'K') // back sensor
	  {
		  compare = 0;
		  TIM2->CCR1 = 0; // sağ teker
		  TIM2->CCR2 = 0; // sol teker
		  TIM3->CCR1 = 0; // sol teker ters
		  TIM3->CCR2 = 0; // sağ teker ters
	  	  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'T') // front sensor
	  {
		  compare = 0;
		  TIM2->CCR1 = 0; // sağ teker
		  TIM2->CCR2 = 0; // sol teker
		  TIM3->CCR1 = 0; // sol teker ters
		  TIM3->CCR2 = 0; // sağ teker ters
	  	  rxIndex++;
	  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);

	  }*/
	  /*else if(rxBuf[rxIndex] == 'O') // reverse stop
	  {
		  stopperB = 1;
		  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'J') // reverse stop
	  {
		  stopperR = 0;
		  rxIndex++;
	  }

	  else if(rxBuf[rxIndex] == 'K') // reverse stop
	  {
		  stopperB = 0;
		  rxIndex++;
	  }*/

	  /*
	  else //emniyet pimi gereksiz
	  {
		  TIM2->CCR1 = 0; // sağ teker
		  TIM2->CCR2 = 0; // sol teker
		  TIM3->CCR1 = 0; // sol teker ters
		  TIM3->CCR2 = 0; // sağ teker ters
	  	  rxIndex++;
		  HAL_Delay(50);
	  }
	  */



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void otoparkL(void){

	 compare = 0;
     TIM2->CCR1 = 0; // sağ teker
	 TIM2->CCR2 = 0; // sol teker
	 TIM3->CCR1 = 0; // sol teker ters
	 TIM3->CCR2 = 0; // sağ teker ters

	 HAL_Delay(1000);

	 //ileri sol yap
	  TIM2->CCR1 = 86;
	  TIM2->CCR2 = 75;
	  for(int k = 0; k < 900; k++)
	  {
	      HAL_Delay(1);
		  TIM2->CCR1 = 86;
		  TIM2->CCR2 = 78;
		  	  if(rxBuf[rxIndex] == 'T')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1500); //dur

	  //düz geri yap
	  TIM3->CCR1 = 86;
	  TIM3->CCR2 = 85;
	  for(int k = 0; k < 1500; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 75;
		  TIM3->CCR2 = 76;
		  	  if(rxBuf[rxIndex] == 'K')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

}


void otoparkP(void){
	  compare = 0;
	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000);

	  //ileri düz yap
	  TIM2->CCR1 = 82;
	  TIM2->CCR2 = 80;
	  for(int k = 0; k < 500; k++)
	  {
	      HAL_Delay(1);
		  TIM2->CCR1 = 82;
		  TIM2->CCR2 = 80;
		  	  if(rxBuf[rxIndex] == 'T')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

	  //sol geri yap

	  TIM3->CCR1 = 87; // sol teker ters
	  TIM3->CCR2 = 75; // sağ teker ters
	  for(int k = 0; k < 300; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 87;
		  TIM3->CCR2 = 75;
		  	  if(rxBuf[rxIndex] == 'K')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

	  //düz geri yap

	  TIM3->CCR1 = 80; // sol teker ters
	  TIM3->CCR2 = 82; // sağ teker ters
	  for(int k = 0; k < 300; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 80;
		  TIM3->CCR2 = 82;
		  	  if(rxBuf[rxIndex] == 'K')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

	  //sağ geri yap

	  TIM3->CCR1 = 60; // sol teker ters
	  TIM3->CCR2 = 87; // sağ teker ters
	  for(int k = 0; k < 280; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 60;
		  TIM3->CCR2 = 87;
		  	  if(rxBuf[rxIndex] == 'K')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

	  //düz geri yap

	  TIM2->CCR1 = 81; // sağ teker
	  TIM2->CCR2 = 80; // sol teker
	  for(int k = 0; k < 80; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 81;
		  TIM3->CCR2 = 80;
		  	  if(rxBuf[rxIndex] == 'K')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }

	  //

	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur



	  //
	  /*
	  TIM2->CCR1 = 0; // sağ teker
	  TIM2->CCR2 = 0; // sol teker
	  TIM3->CCR1 = 0; // sol teker ters
	  TIM3->CCR2 = 0; // sağ teker ters

	  HAL_Delay(1000); //dur

	  //düz geri yap

	  TIM3->CCR1 = 81; // sol teker ters
	  TIM3->CCR2 = 80; // sağ teker ters
	  for(int k = 0; k < 300; k++)
	  {
	      HAL_Delay(1);
		  TIM3->CCR1 = 81;
		  TIM3->CCR2 = 80;
		  	  if(rxBuf[rxIndex] == 'S')
		  	  {
			  TIM2->CCR1 = 0; // sağ teker
			  TIM2->CCR2 = 0; // sol teker
			  TIM3->CCR1 = 0; // sol teker ters
			  TIM3->CCR2 = 0; // sağ teker ters
		  	  }
	  }
      */
	  //

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
