/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024  * in the root directory of this software component.
  *   * in the root di  * in the root directory of this software component.

  *
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rover_msgs/msg/controller_msg.h>
#include <rover_msgs/msg/encoder_msg.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct joystick {
    float x;
    float y;
    int16_t throttle;
    int16_t cameray;  // İleri/Geri ekseni 1 yada -1 yada 0
    int8_t light;
    int8_t camerax;  // Sağ/Sol ekseni 1 yada -1 yada 0
    int16_t imu_speed;// -1000 ile 1000 arasında rover hızı
    uint8_t gear;
} joystick;
joystick rcjoystick = {0, 0, 0, 0, 0, 0, 0, 3};

typedef enum {
    GROUND_DEFAULT,
    GROUND_SLIPPERY,
    GROUND_ROUGH
} GroundType;

GroundType currentGround = GROUND_DEFAULT;

typedef struct {
    int16_t targetSpeed;
    int16_t currentSpeed;
    int16_t previousErrorSpeed;
    int integralSpeed;

	float KP;
	float KI;
	float KD;
	float integralLimit;
} Wheel;

Wheel leftFrontWheel = {0, 0, 0, 0, 0.67f, 0.04f, 0.01f, 1000};//1
Wheel rightFrontWheel = {0, 0, 0, 0, 0.60f, 0.04f, 0.01f, 1000};//2
Wheel leftBackWheel = {0, 0, 0, 0, 0.65f, 0.04f, 0.01f, 1000};//3
Wheel rightBackWheel = {0, 0, 0, 0, 0.65f, 0.04f, 0.01f, 1000};//4

typedef struct {
    int16_t targetDirection;
    int16_t currentDirection;
    int16_t previousErrorDirection;
    int integralDirection;
} Direction;

Direction roverDirection = {90, 0, 0, 0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEGREE_MIN 400
#define DEGREE_MAX 2600
#define DEGREE_STEP 10

#define Direction_KP 0.4f
#define Direction_KI 0.1f
#define Direction_KD 0.03f
#define MAX_SPEED_PWM  1000
#define MIN_SPEED_PWM  -1000

// Motor hızı sınırlayıcı
#define MAX_SPEED_D  1000
#define MAX_SPEED_S  3000
#define MIN_SPEED_D  -1000
#define MAX_DIRECTION_SPEED  300
#define MIN_DIRECTION_SPEED -300


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  htim1.Init.Prescaler = 6;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

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
  htim3.Init.Prescaler = 6;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);




//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<MESAJ ALMA>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//----------------------------------------------------------------------------------------------------------------------

rcl_node_t nodesub;
rcl_subscription_t subscriber;
rcl_subscription_t subscriber2;
int16_t rotation_speedL = 0;/*300 ile -300 arasında olsun*/
int16_t rotation_speedR = 0;/*300 ile -300 arasında olsun*/
int dt;
volatile uint32_t last_heartbeat_time = 0,heartbeat_time = 0,last_message_time;
bool connection_lost = false;


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CAMERA KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
void SetServoPosition(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t* position, uint8_t step, uint8_t servoDirection, uint16_t camera) {
    if (servoDirection > 0) {
        *position = (*position + step <= DEGREE_MAX) ? *position + step : DEGREE_MAX;
    }
    	else if (servoDirection < 0) {
        *position = (*position - step >= DEGREE_MIN) ? *position - step : DEGREE_MIN;
    }
    	else if (servoDirection == 0) {
    	*position = (camera > DEGREE_MAX) ? DEGREE_MAX : (camera < DEGREE_MIN) ? DEGREE_MIN : camera;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, *position);
}

void KameraKontrol()
{
    static uint16_t degreey = 2200;
    static uint16_t degreex = 1500;

    degreex = (degreex > DEGREE_MAX) ? DEGREE_MAX : (degreex < DEGREE_MIN) ? DEGREE_MIN : degreex;
    degreey = (degreey > DEGREE_MAX) ? DEGREE_MAX : (degreey < DEGREE_MIN) ? DEGREE_MIN : degreey;



    		SetServoPosition(&htim2, TIM_CHANNEL_2, &degreey, DEGREE_STEP, 0, rcjoystick.cameray);


    switch (rcjoystick.camerax)
    {
    	case 0: // Dur
    		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, degreex);
    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    		break;

        case 1: // Sağa
            SetServoPosition(&htim2, TIM_CHANNEL_1, &degreex, DEGREE_STEP, 1, rcjoystick.camerax);
            break;

        case -1: // Sola
            SetServoPosition(&htim2, TIM_CHANNEL_1, &degreex, DEGREE_STEP, -1, rcjoystick.camerax);
            break;

        default:
            // Geçersiz joystick z değeri
            break;
    }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MOTOR KONTROL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------------------------------------------------------------------------------------------
/* --- Hız-PWM Uyumluluğu --- */
int16_t speedToPWM(int16_t speed, int16_t max_speed) {
    return speed;
}

/* --- PWM Ayarı --- */
void setPWM(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3, uint32_t channel, int16_t pwmvalue) {



	if (pwmvalue >= 0 && pwmvalue <= 1000) {
        __HAL_TIM_SET_COMPARE(htim1, channel, fabs(pwmvalue));
        __HAL_TIM_SET_COMPARE(htim3, channel, 0);
    }

    else if(pwmvalue >= -1000 && pwmvalue < 0 ) {
        __HAL_TIM_SET_COMPARE(htim1, channel, 0);
        __HAL_TIM_SET_COMPARE(htim3, channel, fabs(pwmvalue));
    }

    else if(pwmvalue > 1000 ) {
        __HAL_TIM_SET_COMPARE(htim1, channel, 1000);
        __HAL_TIM_SET_COMPARE(htim3, channel, 0);
    }

    else if(pwmvalue < -1000 ) {
        __HAL_TIM_SET_COMPARE(htim1, channel, 0);
        __HAL_TIM_SET_COMPARE(htim3, channel, 1000);
    }

    else {

    }
}

/* --- PID Hesaplama --- */
float calculatePID(Wheel *wheel, int max_pwm, int min_pwm, int dt) {


	float error = wheel->targetSpeed-wheel->currentSpeed;
    wheel->integralSpeed += error / dt;
    if (wheel->integralSpeed > wheel->integralLimit) wheel->integralSpeed = max_pwm / wheel->KP;  // Anti-windup
    if (wheel->integralSpeed < -wheel->integralLimit) wheel->integralSpeed = min_pwm / wheel->KP;  // Anti-windup

    float derivative = (error - wheel->previousErrorSpeed) * dt;
    wheel->previousErrorSpeed = error;

    float output = wheel->KP * error + wheel->KI * wheel->integralSpeed + wheel->KD * derivative;

    if (output > max_pwm) output = max_pwm;
    if (output < min_pwm) output = min_pwm;

    return output;
}


/* --- Hız Sınırlandırma --- */
int16_t limitSpeedToPWM(int16_t speed, GroundType groundType, int16_t max_speed) {

	if(rcjoystick.light==1){
		speed=speed*(MAX_SPEED_S/1000);
		if (speed > MAX_SPEED_S) {speed = MAX_SPEED_S;}/*rgb yak*/
	}/*rgb yak*/

	else{
		if (speed > max_speed) {speed = max_speed;}/*rgb yak*/
	}

	if (speed < MIN_SPEED_D) {speed = MIN_SPEED_D;}/*rgb yak*/





	if (speed < MIN_SPEED_D) {speed = MIN_SPEED_D;}/*rgb yak*/


	switch (groundType) {
        case GROUND_SLIPPERY:
            return speedToPWM(speed, max_speed)*0.5f; // Kaygan zemin: %50 hız
        case GROUND_ROUGH:
            return speedToPWM(speed, max_speed)*0.8f; // Engebeli zemin: %80 hız
        default:
            return speedToPWM(speed, max_speed);        // Normal zemin: Tam hız
		}
}

uint64_t now;
/* --- Tekerlek Kontrolü --- */
void controlWheel(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim3, uint32_t channel, Wheel *wheel) {

	now = HAL_GetTick();
	dt = now - last_heartbeat_time;

	int16_t pwmValue = calculatePID(wheel, MAX_SPEED_PWM, MIN_SPEED_PWM, dt);
	last_heartbeat_time = HAL_GetTick();
	setPWM(htim1, htim3, channel, pwmValue);

}

/* --- Yon Kontrolü --- */
void controlDirection(int16_t Y, int16_t X, Direction *direction, int16_t *rotation_speedL, int16_t *rotation_speedR, uint8_t Gear){

	/*direction->targetDirection = atan2(Y,X) * (180 / M_PI) + 180;

    int16_t error = 	direction->targetDirection - direction->currentDirection;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    direction->integralDirection += error;
    int16_t derivative = (error - direction->previousErrorDirection);
    direction->previousErrorDirection = error;

    int16_t output = Direction_KP * error + Direction_KI * direction->integralDirection + Direction_KD * derivative;*/

    int16_t output = X;
    if (output > MAX_DIRECTION_SPEED) output = MAX_DIRECTION_SPEED;
    if (output < MIN_DIRECTION_SPEED) output = MIN_DIRECTION_SPEED;


		*rotation_speedL=-output;
		*rotation_speedR=output;




}


/* --- Rover Kontrol Fonksiyonu --- */
void controlRover(TIM_HandleTypeDef *htim1,TIM_HandleTypeDef *htim3 , int16_t rotation_speedL, int16_t rotation_speedR, uint16_t throttle) {


	    leftFrontWheel.targetSpeed  = (limitSpeedToPWM(throttle+rotation_speedL , currentGround, MAX_SPEED_D));
	    leftBackWheel.targetSpeed   = (limitSpeedToPWM(throttle+rotation_speedL , currentGround, MAX_SPEED_D));
	    rightFrontWheel.targetSpeed = (limitSpeedToPWM(throttle+rotation_speedR , currentGround, MAX_SPEED_D));
	    rightBackWheel.targetSpeed  = (limitSpeedToPWM(throttle+rotation_speedR , currentGround, MAX_SPEED_D));


	    controlWheel(htim1, htim3, TIM_CHANNEL_1, &leftFrontWheel);
	    controlWheel(htim1, htim3, TIM_CHANNEL_2, &rightFrontWheel);
	    controlWheel(htim1, htim3, TIM_CHANNEL_3, &leftBackWheel);
	    controlWheel(htim1, htim3, TIM_CHANNEL_4, &rightBackWheel);



}



void subscription_callback_encoder(const void * msgin)
{
	const rover_msgs__msg__EncoderMsg * incoming_msg = (const rover_msgs__msg__EncoderMsg *)msgin;

	leftFrontWheel.currentSpeed = incoming_msg->m1*2.6f ;
	rightFrontWheel.currentSpeed = incoming_msg->m2*2.6f;
	leftBackWheel.currentSpeed = incoming_msg->m3*2.6f;
	rightBackWheel.currentSpeed = incoming_msg->m4*2.6f;

}


void subscription_callback_controller(const void * msgin){


	// Cast received message to used type
	const rover_msgs__msg__ControllerMsg * incoming_msg = (const rover_msgs__msg__ControllerMsg *)msgin;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    //en son alinan mesajin kacinci saniyede alindigini tut
	last_message_time = HAL_GetTick();


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	rcjoystick.x = (int)incoming_msg->x;
	rcjoystick.y = (int)incoming_msg->y;
	rcjoystick.throttle = (int)incoming_msg->throttle;
	rcjoystick.camerax = (int)incoming_msg->camerax;
	rcjoystick.cameray = (int)incoming_msg->cameray;
	rcjoystick.light = (uint8_t)incoming_msg->light;
	rcjoystick.gear = incoming_msg->gear;
	roverDirection.currentDirection = incoming_msg->yaw;

    // Rover'ı kontrol et
    controlDirection(rcjoystick.y, rcjoystick.x, &roverDirection, &rotation_speedL, &rotation_speedR, rcjoystick.gear);
    controlRover(&htim1, &htim3, rotation_speedL, rotation_speedR, rcjoystick.throttle);
    KameraKontrol();
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  // micro-ROS configuration
	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;



	      rover_msgs__msg__ControllerMsg submsg;
	      rover_msgs__msg__EncoderMsg encodermsg;
	      rclc_support_t support;
	      rclc_executor_t executor;
	      rcl_allocator_t allocator;


	      allocator = rcl_get_default_allocator();

	      	    // create init_options
	      rclc_support_init(&support, 0, NULL, &allocator);


	      	    // create node
	      rclc_node_init_default(&nodesub, "sub_node", "", &support);
	      rclc_executor_init(&executor, &support.context, 2, &allocator);


	      	    // create subscription
	      rclc_subscription_init_default(
	      	        &subscriber,
	      	        &nodesub,
	      	        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, ControllerMsg),
	      	        "joystick_cmd");

	      rclc_subscription_init_default(
	      	      	        &subscriber2,
	      	      	        &nodesub,
	      	      	        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, EncoderMsg),
	      	      	        "encoder");

	      rclc_executor_add_subscription(
	      	      	      &executor, &subscriber2, &encodermsg,
	      	      	      &subscription_callback_encoder, ON_NEW_DATA);



	      rclc_executor_add_subscription(
	      	      &executor, &subscriber, &submsg,
	      	      &subscription_callback_controller, ON_NEW_DATA);




	  while(1)
	  {
		  //mesaj gelip gelmedigini kontrol et
		  rclc_executor_spin_some(&executor,100);

		  //programin calisma zamani
		  heartbeat_time = HAL_GetTick();

		  //250 milisaniye boyunca mesaj alinmamissa motorlari durdur.
		  if (HAL_GetTick() - last_message_time > 250) {
		      connection_lost = true;
		  }
		  else {
		      connection_lost = false;
		  }

		  if(connection_lost == true){
		    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);

		    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  }
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
