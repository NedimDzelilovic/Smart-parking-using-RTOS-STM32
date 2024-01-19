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
#include "cmsis_os.h"
#include "usb_device.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId motorRampaHandle;
osThreadId lightShowHandle;
osThreadId interfonHandle;
osThreadId citacVrataHandle;
osThreadId displejHandle;
osThreadId pomocniHandle;
/* USER CODE BEGIN PV */
uint8_t USBDataReady = 0;
uint8_t* Buffer;
uint16_t buflen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void motorRampa_init(void const * argument);
void lightShow_init(void const * argument);
void interfon_init(void const * argument);
void citacVrata_init(void const * argument);
void displej_init(void const * argument);
void pomocni_init(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USBRxHandler(uint8_t* buf, uint16_t Len){
	USBDataReady = 1;
	Buffer = buf;
	buflen = Len;
}
//motor deklaracija
void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

void stepper_set_rpm(int rpm) // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000 / 4096 / rpm);
}

void stepper_half_drive_gate(int step) {
	switch (step) {
	case 0:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   // IN4
		break;

	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   // IN4
		break;

	case 2:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   // IN4
		break;

	case 3:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   // IN4
		break;

	case 4:

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   // IN4
		break;

	case 5:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // IN4
		break;

	case 6:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // IN4
		break;

	case 7:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // IN4
		break;
	}
}
//motor2
void stepper_half_drive_gate2(int step) {
	switch (step) {
	case 0:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
		break;

	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
		break;

	case 2:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
		break;

	case 3:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
		break;

	case 4:

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // IN4
		break;

	case 5:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
		break;

	case 6:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
		break;

	case 7:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   // IN3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // IN4
		break;
	}
}
void delay_ms_soft(uint32_t ms)
{
	volatile uint32_t k = 10500 * ms;
	while(k--);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		__HAL_RCC_GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
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
  /* definition and creation of motorRampa */
  osThreadDef(motorRampa, motorRampa_init, osPriorityNormal, 0, 128);
  motorRampaHandle = osThreadCreate(osThread(motorRampa), NULL);

  /* definition and creation of lightShow */
  osThreadDef(lightShow, lightShow_init, osPriorityNormal, 0, 128);
  lightShowHandle = osThreadCreate(osThread(lightShow), NULL);

  /* definition and creation of interfon */
  osThreadDef(interfon, interfon_init, osPriorityNormal, 0, 128);
  interfonHandle = osThreadCreate(osThread(interfon), NULL);

  /* definition and creation of citacVrata */
  osThreadDef(citacVrata, citacVrata_init, osPriorityNormal, 0, 128);
  citacVrataHandle = osThreadCreate(osThread(citacVrata), NULL);

  /* definition and creation of displej */
  osThreadDef(displej, displej_init, osPriorityNormal, 0, 128);
  displejHandle = osThreadCreate(osThread(displej), NULL);

  /* definition and creation of pomocni */
  osThreadDef(pomocni, pomocni_init, osPriorityNormal, 0, 128);
  pomocniHandle = osThreadCreate(osThread(pomocni), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xfffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1107-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE6 PE9 PE11
                           PE13 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB4 PB5 PB7
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_motorRampa_init */
/**
  * @brief  Function implementing the motorRampa thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_motorRampa_init */
void motorRampa_init(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  //uint8_t prev_state = 0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    //Otvaranje rampe kada primi poruku open od citaca
	if(USBDataReady == 1 && (strncmp(Buffer,"Open\n",buflen)==0)){
	float anglepersequence = 0.703125;
	uint32_t angle_sequence = (int) (90 / anglepersequence);
	uint32_t temp_angle_sequence = angle_sequence;
	while (temp_angle_sequence) {
		for (int step = 7; step >= 0; step--) {
			stepper_half_drive_gate(step);
			stepper_set_rpm(13);
		}
		temp_angle_sequence--;
	}
	HAL_Delay(3000);
	while (temp_angle_sequence <= angle_sequence) {
		for (int step = 0; step < 8; step++) {
			stepper_half_drive_gate(step);
			stepper_set_rpm(13);
		}
		++temp_angle_sequence;
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 0) {
		HAL_Delay(1000);
		while (temp_angle_sequence) {
			for (int step = 7; step >= 0; step--) {
				stepper_half_drive_gate(step);
				stepper_set_rpm(4);
			}
			temp_angle_sequence--;
		}
		HAL_Delay(3000);
		}
	}

	USBDataReady = 0;
	//Buffer = 0;
	//buflen = 0;
	}
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0){
			float anglepersequence = 0.703125;
			uint32_t angle_sequence = (int) (90 / anglepersequence);
			uint32_t temp_angle_sequence = angle_sequence;
			while (temp_angle_sequence) {
				for (int step = 7; step >= 0; step--) {
					stepper_half_drive_gate(step);
					stepper_set_rpm(13);
				}
				temp_angle_sequence--;
			}
			HAL_Delay(3000);
			while (temp_angle_sequence <= angle_sequence) {
				for (int step = 0; step < 8; step++) {
					stepper_half_drive_gate(step);
					stepper_set_rpm(13);
				}
				++temp_angle_sequence;
				if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 0) {
				HAL_Delay(1000);
				while (temp_angle_sequence) {
					for (int step = 7; step >= 0; step--) {
						stepper_half_drive_gate(step);
						stepper_set_rpm(4);
					}
					temp_angle_sequence--;
				}
				HAL_Delay(3000);
				}
			}
		}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_lightShow_init */
/**
* @brief Function implementing the lightShow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lightShow_init */
void lightShow_init(void const * argument)
{
  /* USER CODE BEGIN lightShow_init */
	uint16_t leds[8] = {0xF,0xE,0xC,0x8,0x0,0x1,0x3,0x7};
	uint8_t n = 0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	  delay_ms_soft(100);
	  		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,leds[n] & 0x01);
	  		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,(leds[n] >> 1) & 0x01);
	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,(leds[n] >> 2) & 0x01);
	  		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,(leds[n] >> 3) & 0x01);
	  		n++;
	  		if(n == 8)
	  			n = 0;
  }
  /* USER CODE END lightShow_init */
}

/* USER CODE BEGIN Header_interfon_init */
/**
* @brief Function implementing the interfon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_interfon_init */
void interfon_init(void const * argument)
{
  /* USER CODE BEGIN interfon_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {
    	htim4.Instance->CCR2 = 20;
    	HAL_Delay(2000);
    	}
    HAL_Delay(2000);
    htim4.Instance->CCR2 = 0;
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
    	float anglepersequence = 0.703125;
    		uint32_t angle_sequence = (int) (40 / anglepersequence);
    		uint32_t temp_angle_sequence = angle_sequence;
    		while (temp_angle_sequence) {
    			for (int step = 0; step < 8; step++) {
    				stepper_half_drive_gate2(step);
    				stepper_set_rpm(13);
    			}
    			temp_angle_sequence--;
    		}
    		HAL_Delay(10000);
    		while (temp_angle_sequence <= angle_sequence) {
    			for (int step = 7; step >= 0; step--) {
    				stepper_half_drive_gate2(step);
    				stepper_set_rpm(13);
    			}
    			++temp_angle_sequence;
     			/*
    			if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 0) {
    			HAL_Delay(1000);

    			while (temp_angle_sequence) {
    				for (int step = 7; step >= 0; step--) {
    					stepper_half_drive_gate2(step);
    					stepper_set_rpm(4);
    				}
    				temp_angle_sequence--;
    			}
    			HAL_Delay(3000);
    			}*/
    		}

    	  }
  }
  /* USER CODE END interfon_init */
}

/* USER CODE BEGIN Header_citacVrata_init */
/**
* @brief Function implementing the citacVrata thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_citacVrata_init */
void citacVrata_init(void const * argument)
{
  /* USER CODE BEGIN citacVrata_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(USBDataReady == 1 && (strncmp(Buffer,"Door\n",buflen)==0)){
    	float anglepersequence = 0.703125;
    	uint32_t angle_sequence = (int) (90 / anglepersequence);
    	uint32_t temp_angle_sequence = angle_sequence;
    	while (temp_angle_sequence) {
    		for (int step = 0; step < 8; step++) {
    			stepper_half_drive_gate2(step);
    			stepper_set_rpm(13);
    		}
    		temp_angle_sequence--;
    	}
    	HAL_Delay(3000);
    	while (temp_angle_sequence <= angle_sequence) {
    		for (int step = 7; step >= 0; step--) {
    			stepper_half_drive_gate2(step);
    			stepper_set_rpm(13);
    		}
    		++temp_angle_sequence;

    		/*
    		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 0) {
    		HAL_Delay(1000);
    		while (temp_angle_sequence) {
    			for (int step = 7; step >= 0; step--) {
    				stepper_half_drive_gate2(step);
    				stepper_set_rpm(4);
    			}
    			temp_angle_sequence--;
    		}
    		HAL_Delay(3000);
    		}*/
    	}

    	USBDataReady = 0;
    	Buffer = 0;
    	buflen=0;
    	}
  }
  /* USER CODE END citacVrata_init */
}

/* USER CODE BEGIN Header_displej_init */
/**
* @brief Function implementing the displej thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displej_init */
void displej_init(void const * argument)
{
  /* USER CODE BEGIN displej_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END displej_init */
}

/* USER CODE BEGIN Header_pomocni_init */
/**
* @brief Function implementing the pomocni thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pomocni_init */
void pomocni_init(void const * argument)
{
  /* USER CODE BEGIN pomocni_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }
  /* USER CODE END pomocni_init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
