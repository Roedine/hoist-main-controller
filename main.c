/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#define SINE_FREQUENCY 10000   // output frequency
#define SYSTEM_CLOCK         16000000UL   // 16 MHz timer clock (HSI, no prescaler)
#define PWM_FREQUENCY        1000UL       // desired PWM frequency, in Hz
#define SINE_FREQUENCY       50UL         // desired sine wave frequency, in Hz
#define SINE_AMPLITUDE 		 1500UL

#define TIMER_PERIOD         15999UL//((SYSTEM_CLOCK / PWM_FREQUENCY) - 1UL)
#define PWM_PERIOD 1000

#define SINE_TABLE_SIZE      1024//256
#define PHASE_INCREMENT      ((uint32_t)( (SINE_FREQUENCY * SINE_TABLE_SIZE * 65536UL) / PWM_FREQUENCY ))

#include <math.h>

// nRF24 Includes //
#include <string.h>
#include <stdio.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"

#define PLD_SIZE 32

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

/* Definitions for FSM */
osThreadId_t FSMHandle;
const osThreadAttr_t FSM_attributes = {
  .name = "FSM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PWMTask */
osThreadId_t PWMTaskHandle;
const osThreadAttr_t PWMTask_attributes = {
  .name = "PWMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for nRF_communicati */
osThreadId_t nRF_communicatiHandle;
const osThreadAttr_t nRF_communicati_attributes = {
  .name = "nRF_communicati",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
volatile uint16_t sine_index = 0;
volatile uint32_t phase_accumulator = 0;
uint16_t sine_table[SINE_TABLE_SIZE];


/* Definitions for Modulation_Mutex */
osMutexId_t Modulation_MutexHandle;
const osMutexAttr_t Modulation_Mutex_attributes = {
  .name = "Modulation_Mutex"
};

/* Definitions for PWM_Semaphore */
osSemaphoreId_t PWM_SemaphoreHandle;
const osSemaphoreAttr_t PWM_Semaphore_attributes = {
  .name = "PWM_Semaphore"
};

enum FSM_states {
	Idle,
	Horizontal,
	Lateral,
	Emergency
};

enum FSM_states fsm_state = Idle;


// Motor control parameters
#define MIN_MOTOR_FREQ    5UL      // Minimum frequency for cold start (Hz)
#define MAX_MOTOR_FREQ    50UL     // Maximum operating frequency (Hz)
#define RAMP_STEPS        10       // Number of steps from min to max frequency

volatile uint32_t target_frequency = 0;  // Target motor frequency
volatile uint32_t current_frequency = 0; // Current motor frequency


// nRF24 PV //
uint32_t last_receive_time = 0;
uint32_t current_time = 0;

struct states {
	uint8_t left;
	uint8_t right;
	uint8_t up;
	uint8_t down;
	uint8_t estop;
};

struct states state = {0};

uint8_t modulation_index[4] = {0};

uint8_t transmit_request = 0;

volatile int8_t motor_direction = 1;  // 1 = forward (ABC), -1 = reverse (ACB)

char tmp[80];
uint8_t one_disconnect = 0;
uint8_t one_emergency = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
void StartFSM(void *argument);
void StartPWMTask(void *argument);
void Start_nRF(void *argument);

/* USER CODE BEGIN PFP */
void handle_button(void);

// nRF24 PFP //
void receive_message(void);
void transmit_message(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// nRF24 Data //
uint8_t data_T[PLD_SIZE] = {"Hello!!!"};
uint8_t ack_T[PLD_SIZE];

uint8_t data_R[PLD_SIZE];
uint8_t ack_R[PLD_SIZE] = {"Received"};

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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

// PWM Initialize //

  for (int i = 0; i < SINE_TABLE_SIZE; i++)
  {
      sine_table[i] = (uint16_t)(1500 + 1500 * sinf(2 * M_PI * i / SINE_TABLE_SIZE));
  }

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


	// nRF24 Initialize //
	  csn_high();
	  nrf24_init();
	  nrf24_tx_pwr(_0dbm);
	  nrf24_data_rate(_1mbps);
	  nrf24_set_channel(78);
	  nrf24_set_crc(en_crc, _1byte);
	  nrf24_pipe_pld_size(0, PLD_SIZE);
	  uint8_t addr[5] = {0x10, 0x21, 0x32, 0x43, 0x54};
	  nrf24_open_tx_pipe(addr);
	  nrf24_open_rx_pipe(0, addr);

	  current_time = HAL_GetTick();

	  nrf24_listen();

	// RTOS begin //

//	osStatus_t status = osKernelInitialize();
//	if (status != osOK) {
//	    Error_Handler();
//	}
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  Modulation_MutexHandle = osMutexNew(&Modulation_Mutex_attributes);
      if (Modulation_MutexHandle == NULL) {
          Error_Handler();
      }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
      PWM_SemaphoreHandle = osSemaphoreNew(1, 0, &PWM_Semaphore_attributes);
          if (PWM_SemaphoreHandle == NULL) {
              Error_Handler();
          }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FSM */
  FSMHandle = osThreadNew(StartFSM, NULL, &FSM_attributes);

  /* creation of PWMTask */
  PWMTaskHandle = osThreadNew(StartPWMTask, NULL, &PWMTask_attributes);

  /* creation of nRF_communicati */
  nRF_communicatiHandle = osThreadNew(Start_nRF, NULL, &nRF_communicati_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 312;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 15000;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 5000;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 160;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 312;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart6.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|CSN_Pin|CE_Pin|IN1_Pin
                          |IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// nRF24 functions //
void receive_message(void)
{
	  if(nrf24_data_available())
	  {
		  nrf24_receive(data_R, sizeof(data_R));


		  state.left = data_R[0] - 65;
		  state.right = data_R[1] - 65;
		  state.up = data_R[2] - 65;
		  state.down = data_R[3] - 65;
		  state.estop = data_R[4] - 65;
		  modulation_index[0] = data_R[5];
		  modulation_index[1] = data_R[6];
		  modulation_index[2] = data_R[7];
		  modulation_index[3] = data_R[8];

		  last_receive_time = HAL_GetTick();

		  for(uint8_t i=0; i<sizeof(data_R); i++)
		  {
			  data_R[i] = '\0';
		  }
	  }
}

void transmit_message(void)
{
//	for(uint8_t i=0; i<sizeof(data_T); i++)
//			  data_T[i] = '\0';
//			data_T

		  nrf24_transmit(data_T, sizeof(data_T));
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFSM */
/**
  * @brief  Function implementing the FSM thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFSM */
void StartFSM(void *argument)
{
  /* USER CODE BEGIN 5 */
	  sprintf(tmp, "Controller Connected");
	  HAL_UART_Transmit(&huart6, (uint8_t*)tmp, strlen(tmp), 200);

  /* Infinite loop */
  for(;;)
  {
	  // Read current button states (protected by implicit RTOS context switching)
	  uint8_t left_active = state.left;
	  uint8_t right_active = state.right;
	  uint8_t up_active = state.up;
	  uint8_t down_active = state.down;
	  uint8_t estop_active = state.estop;

	      // Determine next state based on inputs (priority: E-Stop > Lateral > Horizontal > Idle)
	      if (estop_active) {
	        fsm_state = Emergency;
	      }
	      else if (left_active || right_active) {
	        fsm_state = Lateral;
	      }
	      else if (up_active || down_active) {
	        fsm_state = Horizontal;
	      }
	      else {
	        fsm_state = Idle;
	      }

	      // Execute state actions
	      switch(fsm_state)
	      {
	        case Idle:
	            // Stop motor - reset to cold start condition
	            target_frequency = 0;
	            current_frequency = 0;
	            motor_direction = 1;
	            one_emergency = 0;
	          // Stop all PWM outputs
	          osMutexAcquire(Modulation_MutexHandle, osWaitForever);
	          modulation_index[0] = 0; // Left
	          modulation_index[1] = 0; // Right
	          modulation_index[2] = 0; // Up
	          modulation_index[3] = 0; // Down
	          osMutexRelease(Modulation_MutexHandle);

	          // Turn off all motor channels
	          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

	          // Status LEDs
	            //LEDs
	            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
	            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0);
	            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
	          break;

	        case Horizontal:
	            // Vertical movement (up/down)


	            osMutexAcquire(Modulation_MutexHandle, osWaitForever);

	            uint8_t vertical_mod = 0;
	            if (up_active) {
	              vertical_mod = modulation_index[2];
	              motor_direction = 1;
	            } else if (down_active) {
	              vertical_mod = modulation_index[3];
	              motor_direction = -1;
	            }

	            // Calculate target frequency based on modulation index (0-9)
	            // Ramp from MIN_MOTOR_FREQ to MAX_MOTOR_FREQ
	            if (vertical_mod > 0)
	            {
	              target_frequency = MIN_MOTOR_FREQ + ((MAX_MOTOR_FREQ - MIN_MOTOR_FREQ) * vertical_mod) / 9;
	            }
	            else
	            {
	              target_frequency = MIN_MOTOR_FREQ;
	            }

	            osMutexRelease(Modulation_MutexHandle);

	            // Signal PWM task to update
	            osSemaphoreRelease(PWM_SemaphoreHandle);

	            //LEDs
	            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
	            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1);
	            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

	            break;
	          break;

	        case Lateral:

	            //LEDs
	            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
	            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1);
	            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

	            osMutexAcquire(Modulation_MutexHandle, osWaitForever);

	            uint8_t lateral_mod = 0;

	            if (left_active) {
	              lateral_mod = modulation_index[0] + 5;
	              if((lateral_mod >= 10))
	              {
	            	  lateral_mod = 9;
	              }
	              // Left direction: IN1=HIGH, IN2=LOW
	              HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
	              HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

//	              if (lateral_mod > 0) {
//	                  pwm_duty = (lateral_mod * __HAL_TIM_GET_AUTORELOAD(&htim2)) / 9;
//	                 }

	            } else if (right_active) {
	              lateral_mod = modulation_index[1] + 5;
	              if((lateral_mod >= 10))
	              {
	            	  lateral_mod = 9;
	              }
	              // Right direction: IN1=LOW, IN2=HIGH
	              HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	              HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

//	              if (lateral_mod > 0) {
//	                  pwm_duty = (lateral_mod * __HAL_TIM_GET_AUTORELOAD(&htim2)) / 9;
//	                 }

	            }

	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (lateral_mod * __HAL_TIM_GET_AUTORELOAD(&htim2)) / 9);

//	            // Calculate target frequency based on modulation index (0-9)
//	            if (lateral_mod > 0)
//	            {
//	              target_frequency = MIN_MOTOR_FREQ + ((MAX_MOTOR_FREQ - MIN_MOTOR_FREQ) * lateral_mod) / 9;
//	            }
//	            else
//	            {
//	              target_frequency = MIN_MOTOR_FREQ;
//	            }

	            osMutexRelease(Modulation_MutexHandle);

	            // Signal PWM task to update
	            osSemaphoreRelease(PWM_SemaphoreHandle);
	            break;
	          break;

	        case Emergency:
	            // Emergency stop - immediate shutdown
	            target_frequency = 0;
	            current_frequency = 0;
	            motor_direction = 1;


	            osMutexAcquire(Modulation_MutexHandle, osWaitForever);
	            modulation_index[0] = 0;
	            modulation_index[1] = 0;
	            modulation_index[2] = 0;
	            modulation_index[3] = 0;
	            osMutexRelease(Modulation_MutexHandle);

	            // Immediately stop all PWM channels
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

	            //LEDs
	            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
	            HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0);
	            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);

	            if(!one_emergency && !one_disconnect)
	            {
	  	      	  sprintf(tmp, "Emergency Stop Pressed");
	  	      	  HAL_UART_Transmit(&huart6, (uint8_t*)tmp, strlen(tmp), 200);
	  	      	  one_emergency = 1;
	            }




	            break;
	      }

	      osDelay(20); // FSM runs at ~50Hz
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPWMTask */
/**
* @brief Function implementing the PWMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWMTask */
void StartPWMTask(void *argument)
{
  /* USER CODE BEGIN StartPWMTask */
	  uint32_t phase_increment = 0;
	  uint32_t last_update_time = HAL_GetTick();
  /* Infinite loop */
  for(;;)
		{
	    uint32_t current_update_time = HAL_GetTick();
	    uint32_t delta_time = current_update_time - last_update_time;

	  if (delta_time >= 10)
	      {
		  last_update_time = current_update_time;

	      // Smooth frequency ramping - gradually approach target frequency
	      if (current_frequency < target_frequency)
	      {
	        // Ramp up - increase by 2Hz per 10ms (200Hz/s ramp rate)
	        current_frequency += 2;
	        if (current_frequency > target_frequency)
	        {
	          current_frequency = target_frequency;
	        }
	      }
	      else if (current_frequency > target_frequency)
	            {
	              // Ramp down - decrease by 5Hz per 10ms (faster decel)
	              if (current_frequency > 5)
	              {
	                current_frequency -= 5;
	              }
	              else
	              {
	                current_frequency = 0;
	              }

	              if (current_frequency < target_frequency)
	              {
	                current_frequency = target_frequency;
	              }
	            }
	      }

	      // Calculate phase increment based on current frequency
	      // This updates every loop iteration for smooth sine generation
	      if (current_frequency > 0)
	      {
	        // phase_increment = (frequency * SINE_TABLE_SIZE * 65536) / PWM_FREQUENCY
	        phase_increment = (current_frequency * SINE_TABLE_SIZE * 65536UL) / PWM_FREQUENCY;
	      }
	      else
	      {
	        phase_increment = 0;
	        phase_accumulator = 0; // Reset accumulator when stopped
	      }

	      // Update phase accumulator - this runs at the loop rate (~1kHz or faster)
	          if (phase_increment > 0)
	          {
	            osKernelLock();
	            phase_accumulator += phase_increment;
	            sine_index = (phase_accumulator >> 16) % SINE_TABLE_SIZE;
	            osKernelUnlock();

	            // Calculate three-phase indices (0°, 120°, 240° shift)
	            uint16_t index_u = sine_index;
	            uint16_t index_v = (sine_index + SINE_TABLE_SIZE / 3) % SINE_TABLE_SIZE;
	            uint16_t index_w = (sine_index + 2 * SINE_TABLE_SIZE / 3) % SINE_TABLE_SIZE;

	            // Get timer period for duty cycle calculation
	            uint32_t max_duty = __HAL_TIM_GET_AUTORELOAD(&htim1);

	            // Calculate duty cycles from sine table
	            uint32_t duty_u = (sine_table[index_u] * max_duty) / 3000UL;
	            uint32_t duty_v = (sine_table[index_v] * max_duty) / 3000UL;
	            uint32_t duty_w = (sine_table[index_w] * max_duty) / 3000UL;

	            // V/f control - reduce voltage at lower frequencies
	            uint32_t voltage_scale = 100; // 100% voltage at full frequency

	            if (current_frequency < MAX_MOTOR_FREQ)
	            {
	              // Linear V/f ratio: V = (f / f_rated) * V_rated
	              voltage_scale = (current_frequency * 100) / MAX_MOTOR_FREQ;
	              if (voltage_scale < 30) voltage_scale = 30; // Minimum 30% for startup torque
	            }

	            duty_u = (duty_u * voltage_scale) / 100;
	            duty_v = (duty_v * voltage_scale) / 100;
	            duty_w = (duty_w * voltage_scale) / 100;

	            if (motor_direction > 0) {
	                // Right direction: Normal ABC mapping
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_u);
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_v);
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_w);
	            } else {
	                // Left direction: Swap channels 1 and 2 (ACB mapping)
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_v);
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_u);
	                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_w);
	            }

	          }
	          else
	          {  // Motor stopped
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	          }

	          // Small delay to prevent task hogging CPU (runs at ~1kHz)
	          osDelay(1);
		}
  /* USER CODE END StartPWMTask */
}

/* USER CODE BEGIN Header_Start_nRF */
/**
* @brief Function implementing the nRF_communicati thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_nRF */
void Start_nRF(void *argument)
{
  /* USER CODE BEGIN Start_nRF */
  /* Infinite loop */
  for(;;)
  {

//	  if(HAL_GPIO_ReadPin(Input_GPIO_Port, Input_Pin) == 0)
//	  {
//		  transmit_request = 1;
//	  }

	  switch(transmit_request)
	  {
		  case 1:
			  nrf24_stop_listen();
			  osDelay(10);
			  transmit_message();
			  osDelay(10);
			  nrf24_listen();
			  transmit_request = 0;
			  break;

		  case 0:
			  receive_message();
			  break;
	  }



	  current_time = HAL_GetTick();
	  if(current_time - last_receive_time > 500)
	  {
		  if(!one_disconnect)
		  {
			  sprintf(tmp, "Controller Disconnected");
			  HAL_UART_Transmit(&huart6, (uint8_t*)tmp, strlen(tmp), 200);
			  one_disconnect = 1;
		  }

		  state.left = 0;
		  state.right = 0;
		  state.up = 0;
		  state.down = 0;
		  state.estop = 1;

		  modulation_index[0] = 0;
		  modulation_index[1] = 0;
		  modulation_index[2] = 0;
		  modulation_index[3] = 0;

	  }
	  else
	  {
		  one_disconnect = 0;
	  }


//	  sprintf(tmp, "L:%d %d R:%d %d U:%d %d D:%d %d E:%d\n",
//	          state.left,
//			  modulation_index[0],
//	          state.right,
//			  modulation_index[1],
//	          state.up,
//			  modulation_index[2],
//	          state.down,
//			  modulation_index[3],
//	          state.estop);
//	  HAL_UART_Transmit(&huart6, (uint8_t*)tmp, strlen(tmp), 200);


    osDelay(80);
  }
  /* USER CODE END Start_nRF */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

    if (htim->Instance == TIM1)
    {
        // Signal PWM task to update (interrupt-driven approach)
        // Uncomment this if you want PWM updates synchronized to timer interrupts
        /*
        if (PWM_SemaphoreHandle != NULL) {
          osSemaphoreRelease(PWM_SemaphoreHandle);
        }
        */
    }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
