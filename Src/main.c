/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics and MakerMax Inc.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "oled.h"
#include "adc.h"
#include "soc.h"
#include "sochelper.h"
#include "power.h"
#include "mb.h"
#include "mbport.h"
#include "energy.h"

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
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */

typedef enum
{
	CHG_ENABLE = 1,
	CHG_DISABLE = 0
}CHG_EN;

typedef enum
{
	IDLE = 0,
	CHG = 1,
	DCHG = 2
}STATE;

STATE currentState = IDLE;
uint8_t currentDchgPct = 0;
static float lastReadBattV = 0;
static float lastReadCurr_mA = 0;
static float currentCellSOC = 0;
static float currChargeRemaining = 0; //Ah how much charge is remaining inside the cell
static const float fullChargeCapacity = 3.0; //Ah //TODO: change this as per your cell
static float lastComputedPower = 0;
static float lastComputedEnergy = 0;	//Wh

char powerString[10];
char socString[10];
char currentString[10];
char battVString[10];

union
{
  float     asFloat;
  uint32_t  asUInt32;
}powerData;

union
{
  float     asFloat;
  uint32_t  asUInt32;
}socData;

union
{
  float     asFloat;
  uint32_t  asUInt32;
}battVData;

union
{
  float     asFloat;
  uint32_t  asUInt32;
}currentData;

union
{
  float     asFloat;
  uint32_t  asUInt32;
}energyData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Charging_Enable(CHG_EN chg_en);
void Change_State(STATE new_state);
void Discharging_Set(uint8_t pct);
void Safety_Loop();
void getLatestADCValues();
void updateOLED();
void updateSerialPort();
void updateModbusInputRegisters();
void calcSOC(float ocv_V, float chargeRemain_Ah);
int sprintf(char *out, const char *format, ...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 64
#define REG_HOLDING_START 2000
#define REG_HOLDING_NREGS 8

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	eMBErrorCode    eStatus;

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
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Initialize OLED display
  ssd1306_Init();

  ssd1306_Fill(Black);
  ssd1306_SetCursor(15,04);
  ssd1306_WriteString("BMS", Font_16x26, White);
  ssd1306_UpdateScreen();

  //Initial ADC
  LTC2990_ConfigureControlReg(&hi2c1);

  //Initialize TIM7 to run safety loop
  //HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim16);

  /* ABCDEF */
  usRegInputBuf[0] = 11;
  usRegInputBuf[1] = 22;
  usRegInputBuf[2] = 33;
  usRegInputBuf[3] = 44;
  usRegInputBuf[4] = 55;
  usRegInputBuf[5] = 66;
  usRegInputBuf[6] = 77;
  usRegInputBuf[7] = 88;

  eStatus = eMBInit( MB_RTU, 0x0A, 0, 38400, MB_PAR_NONE );

  /* Enable the Modbus Protocol Stack. */
  eStatus = eMBEnable();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	 getLatestADCValues();
//	 calcSOC(lastReadBattV, currChargeRemaining);
//	 lastComputedPower = computePower(lastReadBattV);
//	 updateOLED();
//	 updateModbusInputRegisters();
//	 //updateSerialPort();
//	 //HAL_Delay(1000); //Update rate to 1s
//
//	 //Modbus Poll routine
//	 eMBPoll();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_TIM17;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 36000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 36000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2500;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 36000;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2500;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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

  //HAL_UART_Transmit(&huart1, (uint8_t *)'I' , 1, 10);

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin|LED_USR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED_USR2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_USR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHG_EN_Pin LED_USR1_Pin */
  GPIO_InitStruct.Pin = CHG_EN_Pin|LED_USR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_INTERRUPT_Pin S1_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = S2_INTERRUPT_Pin|S1_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6){
		HAL_GPIO_TogglePin(LED_USR1_GPIO_Port, LED_USR1_Pin); //LED1 toggles every 0.5 seconds

		//If both buttons S1 and S1 pressed
		if(HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			if(currentState == IDLE)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_SET);
				Change_State(CHG);
			}
			else if(currentState == CHG)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_SET);
				Change_State(DCHG);
			}
			else if(currentState == DCHG)
			{
				HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_RESET);
				Change_State(IDLE);
			}
			else
			{//This should not trigger
				Change_State(IDLE);
			}
		}
		//If either S2 or S1 is pressed while in DCHG state, then change discharge current
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			uint8_t newDchgPct = currentDchgPct + 10;
			//Increase discharge current
			Discharging_Set(newDchgPct);
		}
		else if(currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET)
		{
			uint8_t newDchgPct = currentDchgPct - 10;
			//Decrease discharge current
			Discharging_Set(newDchgPct);
		}

		HAL_TIM_Base_Stop_IT(&htim6);
	}
	/*else if(htim == &htim7){ //Used for modbus sorry!!!
		//Safety_Loop(); //Safety loop run every 0.5 seconds
	}*/
	else if(htim == &htim16){ //This timer ticks every one second and is used for charge remaning calculation
		if(currentState == DCHG || currentState == CHG){
			// SUpply charge remaning with updated current
			currChargeRemaining += calcdeltaAh(1, lastReadCurr_mA / 1000.0);
		}
	}
	else if(htim == &htim17){
		// This timer ticks every 10 seconds and is used for polarization calculations
		//If state is idle
		//Start counting rest time
	}
}

void getLatestADCValues(){

	float battV_2 = 0;
	float voltageADCVcc = 0;

	//Trigger a new conversion
	LTC2990_Trigger(&hi2c1);
	LTC2990_WaitForConversion(&hi2c1, 100);

	//Quick ADC test - Read Vcc
	LTC2990_ReadVoltage(&hi2c1, VCC, &voltageADCVcc);

	//Current reading
	LTC2990_ReadVoltage(&hi2c1, BATTV, &lastReadBattV);
	LTC2990_ReadVoltage(&hi2c1, BATTV_2, &battV_2);
    LTC2990_ReadCurrent(&hi2c1, lastReadBattV, battV_2, &lastReadCurr_mA);
}


void updateOLED(){

	//State
	ssd1306_SetCursor(15,04);
	ssd1306_WriteString("State  ", Font_7x10, White);
	ssd1306_SetCursor(52,04);
	if(currentState == IDLE){
		ssd1306_WriteString("IDLE", Font_7x10, White);
	} else if(currentState == CHG){
		ssd1306_WriteString("CHG", Font_7x10, White);
	} else if(currentState == DCHG){
		ssd1306_WriteString("DCHG", Font_7x10, White);
	} else {
		ssd1306_WriteString(" ", Font_7x10, White);
	}

	//Power
	sprintf(powerString, "%.2d W", (int)lastComputedPower);
	ssd1306_SetCursor(15,14);
	ssd1306_WriteString("Pwr   ", Font_7x10, White);
	ssd1306_SetCursor(52,14);
	ssd1306_WriteString(powerString, Font_7x10, White);

	//SOC
	sprintf(socString, "%.2d ", (int)currentCellSOC);
	ssd1306_SetCursor(15,25);
	ssd1306_WriteString("SOC  ", Font_7x10, White);
	ssd1306_SetCursor(52,25);
	ssd1306_WriteString(socString, Font_7x10, White);


	//Current
	sprintf(currentString, "%.3d mA", (int)lastReadCurr_mA);
	ssd1306_SetCursor(15,37);
	ssd1306_WriteString("Cur  ", Font_7x10, White);
	ssd1306_SetCursor(52,37);
	ssd1306_WriteString(currentString, Font_7x10, White);

	//Voltage
	sprintf(battVString, "%.3d V", (int)lastReadBattV);
	ssd1306_SetCursor(15,49);
	ssd1306_WriteString("BattV   ", Font_7x10, White);
	ssd1306_SetCursor(52,49);
	ssd1306_WriteString(battVString, Font_7x10, White);

	ssd1306_UpdateScreen();

}

void updateSerialPort()
{
	//state
	if(currentState == IDLE){
		HAL_UART_Transmit(&huart1, (uint8_t*)"IDLE", 4, 10);
	} else if(currentState == CHG){
		HAL_UART_Transmit(&huart1, (uint8_t*)"CHG", 3, 10);
	} else if(currentState == DCHG){
		HAL_UART_Transmit(&huart1, (uint8_t*)"DCHG", 4, 10);
	} else {
		HAL_UART_Transmit(&huart1, (uint8_t*)" ", 1, 10);
	}

	//Power
	char powerString[10];
	sprintf(powerString, "%.2d W", (int)lastComputedPower);
	HAL_UART_Transmit(&huart1, (uint8_t*)powerString , 10, 10);

	//SOC
	char socString[10];
	sprintf(socString, "%.2d",(int)currentCellSOC);
	HAL_UART_Transmit(&huart1, (uint8_t*)socString , 10, 10);

	//Current
	char currentString[10];
	sprintf(currentString, "%.3d mA", (int)lastReadCurr_mA);
	HAL_UART_Transmit(&huart1, (uint8_t*)currentString , 10, 10);

	//Voltage
	char voltageString[10];
	sprintf(voltageString, "%.3d V", (int)lastReadBattV);
	HAL_UART_Transmit(&huart1, (uint8_t*)voltageString , 10, 10);

}

void updateModbusInputRegisters()
{
	/* ABCDEF */
	powerData.asFloat = lastComputedPower;
	socData.asFloat = currentCellSOC;
	currentData.asFloat = lastReadCurr_mA;
	battVData.asFloat = lastReadBattV;
	energyData.asFloat = lastComputedEnergy;

	usRegInputBuf[0] = currentState;
	usRegInputBuf[1] = powerData.asUInt32 >> 16;
	usRegInputBuf[2] = powerData.asUInt32;
	usRegInputBuf[3] = socData.asUInt32 >> 16;
	usRegInputBuf[4] = socData.asUInt32;
	usRegInputBuf[5] = currentData.asUInt32 >> 16;
	usRegInputBuf[6] = currentData.asUInt32;
	usRegInputBuf[7] = battVData.asUInt32 >> 16;
	usRegInputBuf[8] = battVData.asUInt32;
	usRegInputBuf[9] = energyData.asUInt32 >> 16;
	usRegInputBuf[10] = energyData.asUInt32;
}

void calcSOC(float ocv_V, float charrgeRemain_Ah){
	//Simple switch to start with
	if(currentState == IDLE){
		//If cell is not polarized.
		currentCellSOC = socByOCV(ocv_V);
		currChargeRemaining = (currentCellSOC / 100) * fullChargeCapacity;
	} else {
		//If cell is polarized
		currentCellSOC = (currChargeRemaining/fullChargeCapacity) * 100;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == S2_INTERRUPT_Pin || GPIO_Pin == S1_INTERRUPT_Pin)
	{
		//S2 has been pressed, start software debouncing
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

//Where pct should be 0 - 100
void Discharging_Set(uint8_t pct)
{
	if(pct < 0)
	{
		pct = 0;
	}
	else if (pct > 100)
	{
		pct = 100;
	}
	//DAC is 12 bit resolution - 0 - 4095 data codes which translates to 0 - 3.3V analog

	uint32_t dacCode = (uint32_t)(( pct / 100.0 ) * 4095.0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacCode);

	currentDchgPct = pct;

	//Start DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Charging_Enable(CHG_EN chg_en)
{
	if(chg_en == CHG_ENABLE)
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_SET);
	}
	else //Disabling charging
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_RESET);
	}
}

void Change_State(STATE new_state)
{
	currentState = new_state;

	//IDLE
	if(currentState == IDLE)
	{
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//CHARGING
	else if(currentState == CHG)
	{
		Charging_Enable(CHG_ENABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//DISCHARGING
	else if (currentState == DCHG)
	{
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(10); //Set discharge current to 10%
	}
	else
	{
		//HANDLE DEFAULT CASE - MISRA C
	}
}

void Safety_Loop()
{
	//Undervoltage - 2.8V
	if(lastReadBattV < 2.8)
	{
		//Stop charging, stop discharging
		//Change_State(IDLE);
	}

	//Overvoltage

	//Overtemperature

	//Overcurrent
}

//16 bit input register data type
//NOTE: "mbpoll -m rtu -a 10 -r 1000 -c 8 -t 3 -b 38400 -d 8 -P NONE /dev/ttyUSB0"
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

//mbpoll -m rtu -a 10 -r 2000 -t 4 -b 38400 -d 8 -P NONE /dev/ttyUSB0 4 5 6
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  getLatestADCValues();
	  calcSOC(lastReadBattV, currChargeRemaining);
	  lastComputedPower = computePower(lastReadBattV);

	  setupEnergyAvailableDischarge(currentCellSOC);

	  if(calculateEnergyAvailableDischarge() == 1)
		  lastComputedEnergy = getEnergyAvaialbelDischarge();

	  //updateOLED();
	  updateModbusInputRegisters();
	  //updateSerialPort();
	  //HAL_Delay(1000); //Update rate to 1s
	  //Modbus Poll routine
	  eMBPoll();
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
