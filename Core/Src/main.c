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
#include "string.h"
#include "stdio.h"
#include "LCD_Driver.h"
#include <math.h>
#include "touch.h"
#include "stdint.h"
#include "music_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_LCD_IDLE,
  STATE_TOUCH_MENU,
  STATE_LED_BLINK,
  STATE_CAN_COMM,
  STATE_CAN_DISPLAY,
  STATE_DAC_ALERT
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

osThreadId defaultTaskHandle;
osThreadId Task02Handle;
osThreadId Task02_1Handle;
osThreadId Task02_2Handle;
osThreadId Task02_3Handle;
osThreadId Task02_4Handle;
/* USER CODE BEGIN PV */
osMutexId spiMutexHandle;
osMutexId lcdMutexHandle;
osMutexId touchMutexHandle;
osMutexId screenMutexHandle;

uint16_t x = 0, y = 0;
volatile uint8_t screenTask = 1;
volatile uint8_t needRefresh = 1;
volatile uint8_t toggleLed = 0;
volatile uint8_t canState = 0;
volatile uint8_t playOn = 0;
volatile uint8_t displayTask02_3 = 0;

CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

#define GOTO_BTN_X1 20
#define GOTO_BTN_X2 220
#define GOTO_BTN_Y1 240
#define GOTO_BTN_Y2 310

#define BACK_BTN_X1 150
#define BACK_BTN_X2 230
#define BACK_BTN_Y1 260
#define BACK_BTN_Y2 300

#define TASK21_BTN_X1 10
#define TASK21_BTN_X2 110
#define TASK21_BTN_Y1 60
#define TASK21_BTN_Y2 140

#define TASK22_BTN_X1 130
#define TASK22_BTN_X2 230
#define TASK22_BTN_Y1 60
#define TASK22_BTN_Y2 140

#define TASK23_BTN_X1 10
#define TASK23_BTN_X2 110
#define TASK23_BTN_Y1 160
#define TASK23_BTN_Y2 240

#define TASK24_BTN_X1 130
#define TASK24_BTN_X2 230
#define TASK24_BTN_Y1 160
#define TASK24_BTN_Y2 240

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const *argument);
void StartTask02(void const *argument);
void StartTask02_1(void const *argument);
void StartTask02_2(void const *argument);
void StartTask02_3(void const *argument);
void StartTask02_4(void const *argument);

/* USER CODE BEGIN PFP */
void Can1WriteData(uint16_t ID, uint8_t *TxData);
void NVIC_Config(void);
void LCD_DisplayTaskDefault(void);
void LCD_DisplayTask2(void);
void LCD_UpdateTask02_1(void);
void LCD_UpdateTask02_2(void);
uint8_t Read_Temperature(void);
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  tp_init();
  tp_adjust();
  NVIC_Config();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(spiMutex);
  spiMutexHandle = osMutexCreate(osMutex(spiMutex));
  osMutexDef(lcdMutex);
  lcdMutexHandle = osMutexCreate(osMutex(lcdMutex));
  osMutexDef(touchMutex);
  touchMutexHandle = osMutexCreate(osMutex(touchMutex));
  osMutexDef(screenMutex);
  screenMutexHandle = osMutexCreate(osMutex(screenMutex));
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 512);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task02_1 */
  osThreadDef(Task02_1, StartTask02_1, osPriorityLow, 0, 256);
  Task02_1Handle = osThreadCreate(osThread(Task02_1), NULL);

  /* definition and creation of Task02_2 */
  osThreadDef(Task02_2, StartTask02_2, osPriorityLow, 0, 256);
  Task02_2Handle = osThreadCreate(osThread(Task02_2), NULL);

  /* definition and creation of Task02_3 */
  osThreadDef(Task02_3, StartTask02_3, osPriorityLow, 0, 256);
  Task02_3Handle = osThreadCreate(osThread(Task02_3), NULL);

  /* definition and creation of Task02_4 */
  osThreadDef(Task02_4, StartTask02_4, osPriorityHigh, 0, 256);
  Task02_4Handle = osThreadCreate(osThread(Task02_4), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef canFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 1;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. Kích hoạt interrupt nhận dữ liệu FIFO0
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
  CAN_FilterTypeDef canFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  canFilterConfig.FilterBank = 14;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader2.StdId = 0x123;
  TxHeader2.ExtId = 0x01;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.DLC = 1;
  TxHeader2.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
   */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 21;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin | LCD_RST_Pin | LCD_CS_Pin | LCD_RS_Pin | T_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin LCD_RST_Pin LCD_BL_Pin LCD_CS_Pin
                           LCD_RS_Pin T_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin | LCD_RST_Pin | LCD_BL_Pin | LCD_CS_Pin | LCD_RS_Pin | T_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_CS_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void NVIC_Config(void)
{

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
  }
}
void Can1WriteData(uint16_t ID, uint8_t *TxData)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  TxHeader.StdId = ID;
  TxHeader.ExtId = 0x00;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    Error_Handler();
  }
}
uint8_t Read_Temperature(void)
{
  uint32_t raw_adc = 0;
  float temperature = 0;
  ADC->CCR |= ADC_CCR_TSVREFE;

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
  {
    raw_adc = HAL_ADC_GetValue(&hadc1);
  }

  HAL_ADC_Stop(&hadc1);

  uint16_t *TS_CAL1 = (uint16_t *)0x1FFF7A2C;
  uint16_t *TS_CAL2 = (uint16_t *)0x1FFF7A2E;

  temperature = ((110.0f - 30.0f) * (raw_adc - *TS_CAL1)) / ((float)(*TS_CAL2 - *TS_CAL1)) + 30.0f;
  return (uint8_t)roundf(temperature);
}

void LCD_DisplayTaskDefault(void)
{
  lcd_clear_screen(WHITE);

  lcd_fill_rect(20, 20, 200, 40, BRED);
  lcd_display_string(70, 30, (uint8_t *)"TASK DEFAULT", FONT_GB2312, WHITE);

  lcd_fill_rect(60, 70, 120, 40, GRAY);
  lcd_display_string(95, 80, (uint8_t *)"NHOM 04", FONT_GB2312, WHITE);

  lcd_display_string(20, 120, (uint8_t *)"MSSV      HO Va TEN", FONT_GB2312, BLACK);
  lcd_display_string(20, 140, (uint8_t *)"22200111  Dang Hoai Nhan", FONT_GB2312, BLACK);
  lcd_display_string(20, 160, (uint8_t *)"22200164  Pham Ngoc Tram", FONT_GB2312, BLACK);
  lcd_display_string(20, 180, (uint8_t *)"22200101  Dao Truc Mai", FONT_GB2312, BLACK);
  lcd_display_string(20, 200, (uint8_t *)"22200173  Nguyen Van Truong", FONT_GB2312, BLACK);

  lcd_fill_rect(20, 240, 200, 70, GREEN);
  lcd_display_string(90, 265, (uint8_t *)"NEXT >>", FONT_GB2312, WHITE);
}

void LCD_DisplayTask2(void)
{
  lcd_clear_screen(WHITE);
  // Group name at top-left
  lcd_fill_rect(10, 10, 100, 30, GRAY);
  lcd_display_string(20, 17, (uint8_t *)"NHOM 04", FONT_GB2312, WHITE);

  // Task 2_1
  //    lcd_fill_rect(10, 60, 100, 80, BLUE);
  lcd_fill_rect(10, 60, 100, 80, toggleLed ? GREEN : BLUE);
  lcd_display_string(30, 90, (uint8_t *)"TASK 2_1", FONT_GB2312, WHITE);

  // Task 2_2
  lcd_fill_rect(130, 60, 100, 80, BLUE);
  lcd_display_string(150, 90, (uint8_t *)"TASK 2_2", FONT_GB2312, WHITE);

  // Task 2_3
  lcd_fill_rect(10, 160, 100, 80, BLUE);
  lcd_display_string(30, 190, (uint8_t *)"TASK 2_3", FONT_GB2312, WHITE);

  // Task 2_4
  lcd_fill_rect(130, 160, 100, 80, BLUE);
  lcd_display_string(150, 190, (uint8_t *)"TASK 2_4", FONT_GB2312, WHITE);

  // "Back" button at bottom-right
  lcd_fill_rect(150, 260, 80, 40, GRAY);
  lcd_display_string(160, 270, (uint8_t *)"<< BACK", FONT_GB2312, WHITE);
}

void LCD_UpdateTask02_1(void)
{
  // Chỉ vẽ lại ô Task02_1
  lcd_fill_rect(10, 60, 100, 80, toggleLed ? GREEN : BLUE);
  lcd_display_string(30, 90, (uint8_t *)"TASK 2_1", FONT_GB2312, WHITE);
}
void LCD_UpdateTask02_2(void)
{
  lcd_fill_rect(130, 60, 100, 80, canState ? GREEN : BLUE);
  lcd_display_string(150, 90, (uint8_t *)"TASK 2_2", FONT_GB2312, WHITE);
}

void LCD_UpdateTask02_4(void)
{
  lcd_fill_rect(130, 160, 100, 80, playOn ? GREEN : BLUE);
  lcd_display_string(150, 190, (uint8_t *)"TASK 2_4", FONT_GB2312, WHITE);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN 5 */
  osThreadSuspend(Task02_1Handle);
  osThreadSuspend(Task02_2Handle);
  osThreadSuspend(Task02_3Handle);
  osThreadSuspend(Task02_4Handle);
  /* Infinite loop */
  for (;;)
  {
    osMutexWait(screenMutexHandle, osWaitForever);
    if (screenTask == 1)
    {
      osMutexRelease(screenMutexHandle);
      osMutexWait(lcdMutexHandle, osWaitForever);
      if (needRefresh)
      {
        needRefresh = 0;
        LCD_DisplayTaskDefault();
      }
      osMutexRelease(lcdMutexHandle);

      osMutexWait(touchMutexHandle, osWaitForever);
      tp_scan(0);
      tp_get_xy(&x, &y);
      osMutexRelease(touchMutexHandle);
      if (x > GOTO_BTN_X1 && x < GOTO_BTN_X2 && y > GOTO_BTN_Y1 && y < GOTO_BTN_Y2 && screenTask == 1)
      {
        osMutexWait(screenMutexHandle, osWaitForever);
        screenTask = 2;
        needRefresh = 1;
        osMutexRelease(screenMutexHandle);
      }
    }
    else
    {
      osMutexRelease(screenMutexHandle);
    }
    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the Task02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for (;;)
  {
    osMutexWait(screenMutexHandle, osWaitForever);
    if (screenTask == 2)
    {
      osMutexRelease(screenMutexHandle);
      osMutexWait(lcdMutexHandle, osWaitForever);
      if (needRefresh)
      {
        needRefresh = 0;
        LCD_DisplayTask2();
      }
      osMutexRelease(lcdMutexHandle);

      osMutexWait(touchMutexHandle, osWaitForever);
      tp_scan(0);
      tp_get_xy(&x, &y);
      osMutexRelease(touchMutexHandle);
      if (x != 0 && y != 0)
        HAL_Delay(200);
      //		BACK
      if (x > BACK_BTN_X1 && x < BACK_BTN_X2 && y > BACK_BTN_Y1 && y < BACK_BTN_Y2 && screenTask == 2)
      {
        osMutexWait(screenMutexHandle, osWaitForever);
        screenTask = 1;
        needRefresh = 1;
        osMutexRelease(screenMutexHandle);
      }
      //		Task02_1
      if (x > TASK21_BTN_X1 && x < TASK21_BTN_X2 && y > TASK21_BTN_Y1 && y < TASK21_BTN_Y2 && screenTask == 2)
      {
        toggleLed = !toggleLed;
        if (toggleLed)
          osThreadResume(Task02_1Handle);
        else
        {
          osThreadSuspend(Task02_1Handle);
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        osMutexWait(lcdMutexHandle, osWaitForever);
        LCD_UpdateTask02_1();
        osMutexRelease(lcdMutexHandle);
      }
      //		Task02_2
      if (x > TASK22_BTN_X1 && x < TASK22_BTN_X2 && y > TASK22_BTN_Y1 && y < TASK22_BTN_Y2 && screenTask == 2)
      {
        canState = !canState;
        if (canState)
          osThreadResume(Task02_2Handle);
        else
          osThreadSuspend(Task02_2Handle);
        osMutexWait(lcdMutexHandle, osWaitForever);
        LCD_UpdateTask02_2();
        osMutexRelease(lcdMutexHandle);
      }
      //		Task02_3
      if (x > TASK23_BTN_X1 && x < TASK23_BTN_X2 && y > TASK23_BTN_Y1 && y < TASK23_BTN_Y2 && screenTask == 2)
      {
        osThreadResume(Task02_3Handle);
      }
      //		Task02_4
      if (x > TASK24_BTN_X1 && x < TASK24_BTN_X2 && y > TASK24_BTN_Y1 && y < TASK24_BTN_Y2 && screenTask == 2)
      {
        playOn = !playOn;
        if (playOn)
          osThreadResume(Task02_4Handle);
        else
          osThreadSuspend(Task02_4Handle);
        osMutexWait(lcdMutexHandle, osWaitForever);
        LCD_UpdateTask02_4();
        osMutexRelease(lcdMutexHandle);
      }
    }
    else
      osMutexRelease(screenMutexHandle);
    osDelay(20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask02_1 */
/**
 * @brief Function implementing the Task02_1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02_1 */
void StartTask02_1(void const *argument)
{
  /* USER CODE BEGIN StartTask02_1 */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartTask02_1 */
}

/* USER CODE BEGIN Header_StartTask02_2 */
/**
 * @brief Function implementing the Task02_2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02_2 */
void StartTask02_2(void const *argument)
{
  /* USER CODE BEGIN StartTask02_2 */
  for (;;)
  {
    uint8_t temperature = Read_Temperature();
    char temp_str[4];
    snprintf(temp_str, sizeof(temp_str), "%02d", temperature);

    uint8_t txData[8] = {'N', '0', '4', ':', ' ', temp_str[0], temp_str[1], 'C'};
    Can1WriteData(0x123, txData);

    osDelay(500);
  }
  /* USER CODE END StartTask02_2 */
}

/* USER CODE BEGIN Header_StartTask02_3 */
/**
 * @brief Function implementing the Task02_3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02_3 */
void StartTask02_3(void const *argument)
{
  /* USER CODE BEGIN StartTask02_3 */
  /* Infinite loop */
  uint8_t localRxData[8];
  char msg[32];

  for (;;)
  {
    // Copy CAN data with mutex protection
    osMutexWait(screenMutexHandle, osWaitForever);
    memcpy(localRxData, RxData, sizeof(RxData));
    osMutexRelease(screenMutexHandle);

    // Get LCD mutex before updating display
    osMutexWait(lcdMutexHandle, osWaitForever);

    // Clear screen and display header
    lcd_clear_screen(WHITE);
    lcd_fill_rect(20, 20, 200, 40, GREEN);
    lcd_display_string(70, 30, (uint8_t *)"TASK 02_3", FONT_GB2312, WHITE);

    // Check if we have valid data
    if (localRxData[0] != 0)
    {
      // Format and display CAN data
      snprintf(msg, sizeof(msg), "DATA: %c%c%c%c%c%c%c%c",
               localRxData[0], localRxData[1], localRxData[2], localRxData[3],
               localRxData[4], localRxData[5], localRxData[6], localRxData[7]);
      lcd_display_string(20, 100, (uint8_t *)msg, FONT_GB2312, BLACK);
    }
    else
    {
      lcd_display_string(20, 100, (uint8_t *)"Waiting for CAN data...", FONT_GB2312, RED);
    }

    osMutexRelease(lcdMutexHandle);

    // Wait for 3 seconds
    osDelay(3000);

    // Return to default screen
    osMutexWait(screenMutexHandle, osWaitForever);
    screenTask = 1;
    needRefresh = 1;
    osMutexRelease(screenMutexHandle);

    // Suspend this task until next time it's needed
    osThreadSuspend(NULL);
  }
  /* USER CODE END StartTask02_3 */
}

/* USER CODE BEGIN Header_StartTask02_4 */
/**
 * @brief Function implementing the Task02_4 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02_4 */
void StartTask02_4(void const *argument)
{
  /* USER CODE BEGIN StartTask02_4 */
  /* Infinite loop */
  for (;;)
  {
    if (playOn)
    {
      // Start playback if not already playing
      HAL_TIM_Base_Start(&htim6); // Start Timer 6 for DAC trigger
      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)&music_data[44], music_len - 44, DAC_ALIGN_8B_R);
    }
    else
    {
      // Stop playback if playing
      HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
      HAL_TIM_Base_Stop(&htim6);
    }
    osDelay(10);
  }
  /* USER CODE END StartTask02_4 */
}

// Add DAC callback
void HAL_DAC_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC)
  {
    // When playback completes, stop everything and reset playOn
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_2);
    HAL_TIM_Base_Stop(&htim6);
    playOn = 0; // Reset playOn when music finishes

    // Update LCD to show stopped state
    osMutexWait(lcdMutexHandle, osWaitForever);
    LCD_UpdateTask02_4();
    osMutexRelease(lcdMutexHandle);
  }
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
  if (htim->Instance == TIM1)
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
  while (1)
  {
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
