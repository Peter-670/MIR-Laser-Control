/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "i2c.h"
#include "mcp4725.h"
#include "usart.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  char text[160];
} UartLogMsg_t;

typedef struct
{
  uint32_t dac1;
  uint32_t dac2;
  uint16_t dac3;
  uint8_t temp1_ready;
  uint8_t temp2_ready;
  uint8_t temp3_ready;
  uint8_t en1;
  uint8_t en2;
  uint8_t en3;
} LaserState_t;

typedef struct
{
  uint16_t raw[4];
} AdcSnapshot_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_FLAG_READY             (1UL << 0)

#define TEMP_DEBOUNCE_MS           200U
#define ADC_DMA_CHANNELS           4U

#define LASER12_DAC_MAX            3103U
#define LASER3_DAC_MAX             1861U

#define LASER1_BIG_STEP            31U
#define LASER1_SMALL_STEP          3U
#define LASER2_BIG_STEP            31U
#define LASER2_SMALL_STEP          3U
#define LASER3_STEP                20U
#define LASER3_SETTLE_DELAY_MS     200U

#define LASER3_VOLTAGE_MIN         2.0f
#define LASER3_VOLTAGE_MAX         3.3f
#define LASER3_ADC_VOLTAGE_MIN     0.0f
#define LASER3_ADC_VOLTAGE_MAX     3.26f
#define LASER3_CURRENT_MIN         0.8f
#define LASER3_CURRENT_MAX         10.0f
#define LASER3_ADC_CURRENT_MIN     0.0f
#define LASER3_ADC_CURRENT_MAX     2.71f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SAT_ADD_U32(v, step, hi) (((v) >= ((hi) - (step))) ? (hi) : ((v) + (step)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static LaserState_t g_laser = {
  .dac1 = 0U,
  .dac2 = 0U,
  .dac3 = 0U,
  .temp1_ready = 0U,
  .temp2_ready = 0U,
  .temp3_ready = 1U,
  .en1 = 0U,
  .en2 = 0U,
  .en3 = 0U
};

static AdcSnapshot_t g_adc = { .raw = {0U, 0U, 0U, 0U} };
static uint16_t g_adc_dma_buf[ADC_DMA_CHANNELS] = {0U};

static osMessageQueueId_t uartTxQueueHandle = NULL;
static osSemaphoreId_t adcReqSemHandle = NULL;
static osSemaphoreId_t adcDoneSemHandle = NULL;
static osSemaphoreId_t adcCpltSemHandle = NULL;
static osEventFlagsId_t appFlagsHandle = NULL;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartRxTask */
osThreadId_t UartRxTaskHandle;
const osThreadAttr_t UartRxTask_attributes = {
  .name = "UartRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TempMonitorTask */
osThreadId_t TempMonitorTaskHandle;
const osThreadAttr_t TempMonitorTask_attributes = {
  .name = "TempMonitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTxTask */
osThreadId_t UartTxTaskHandle;
const osThreadAttr_t UartTxTask_attributes = {
  .name = "UartTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartCmdQueue */
osMessageQueueId_t uartCmdQueueHandle;
const osMessageQueueAttr_t uartCmdQueue_attributes = {
  .name = "uartCmdQueue"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for laserStateMutex */
osMutexId_t laserStateMutexHandle;
const osMutexAttr_t laserStateMutex_attributes = {
  .name = "laserStateMutex"
};
/* Definitions for adcDataMutex */
osMutexId_t adcDataMutexHandle;
const osMutexAttr_t adcDataMutex_attributes = {
  .name = "adcDataMutex"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Log_Printf(const char *fmt, ...);
static void Hardware_InitSafe(void);
static void Laser_UpdateEnableLocked(void);
static bool Laser_CommandAllowedLocked(uint8_t cmd);
static void Laser_ProcessCommand(uint8_t cmd);
static void Adc_RequestAndWait(void);
static float Laser3_ADCtoVoltage(uint16_t adc_value);
static float Laser3_ADCtoCurrent(uint16_t adc_value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  static const osMessageQueueAttr_t uartTxQueueAttr = { .name = "uartTxQueue" };
  static const osSemaphoreAttr_t adcReqSemAttr = { .name = "adcReqSem" };
  static const osSemaphoreAttr_t adcDoneSemAttr = { .name = "adcDoneSem" };
  static const osSemaphoreAttr_t adcCpltSemAttr = { .name = "adcCpltSem" };
  static const osEventFlagsAttr_t appFlagsAttr = { .name = "appFlags" };

  uartTxQueueHandle = osMessageQueueNew(12U, sizeof(UartLogMsg_t), &uartTxQueueAttr);
  adcReqSemHandle = osSemaphoreNew(1U, 0U, &adcReqSemAttr);
  adcDoneSemHandle = osSemaphoreNew(1U, 0U, &adcDoneSemAttr);
  adcCpltSemHandle = osSemaphoreNew(1U, 0U, &adcCpltSemAttr);
  appFlagsHandle = osEventFlagsNew(&appFlagsAttr);
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* creation of laserStateMutex */
  laserStateMutexHandle = osMutexNew(&laserStateMutex_attributes);

  /* creation of adcDataMutex */
  adcDataMutexHandle = osMutexNew(&adcDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartCmdQueue */
  uartCmdQueueHandle = osMessageQueueNew(16U, sizeof(uint8_t), &uartCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UartRxTask */
  UartRxTaskHandle = osThreadNew(StartTask02, NULL, &UartRxTask_attributes);

  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartTask03, NULL, &CommandTask_attributes);

  /* creation of TempMonitorTask */
  TempMonitorTaskHandle = osThreadNew(StartTask04, NULL, &TempMonitorTask_attributes);

  /* creation of UartTxTask */
  UartTxTaskHandle = osThreadNew(StartTask05, NULL, &UartTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t AdcTaskAttr = {
    .name = "AdcTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal
  };
  (void)osThreadNew(StartTask06, NULL, &AdcTaskAttr);

  if ((uartTxQueueHandle == NULL) || (uartCmdQueueHandle == NULL) ||
      (i2cMutexHandle == NULL) || (laserStateMutexHandle == NULL) || (adcDataMutexHandle == NULL) ||
      (adcReqSemHandle == NULL) || (adcDoneSemHandle == NULL) || (adcCpltSemHandle == NULL) ||
      (appFlagsHandle == NULL) || (defaultTaskHandle == NULL) || (UartRxTaskHandle == NULL) ||
      (CommandTaskHandle == NULL) || (TempMonitorTaskHandle == NULL) || (UartTxTaskHandle == NULL))
  {
    const char *msg = "Error: RTOS object create failed\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 200U);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  Hardware_InitSafe();
  osEventFlagsSet(appFlagsHandle, APP_FLAG_READY);
  Log_Printf("RTOS multi-task init done\r\n");

  for (;;)
  {
    Log_Printf("Heartbeat: rx/cmd/temp/adc/tx running\r\n");
    osDelay(8000U);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the UartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint8_t rx = 0U;
  (void)argument;

  (void)osEventFlagsWait(appFlagsHandle, APP_FLAG_READY, osFlagsWaitAll, osWaitForever);
  for (;;)
  {
    if (HAL_UART_Receive(&huart2, &rx, 1U, 20U) == HAL_OK)
    {
      Log_Printf("RX cmd=%c\r\n", rx);
      if ((rx >= (uint8_t)'0') && (rx <= (uint8_t)'9'))
      {
        (void)osMessageQueuePut(uartCmdQueueHandle, &rx, 0U, 0U);
      }
      else
      {
        Log_Printf("Error: invalid cmd (0-9)\r\n");
      }
    }
    osDelay(1U);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the CommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint8_t cmd = 0U;
  (void)argument;

  (void)osEventFlagsWait(appFlagsHandle, APP_FLAG_READY, osFlagsWaitAll, osWaitForever);
  for (;;)
  {
    if (osMessageQueueGet(uartCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK)
    {
      Laser_ProcessCommand(cmd);
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the TempMonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint8_t t1, t2, t3;
  uint8_t old1, old2, old3;
  (void)argument;

  (void)osEventFlagsWait(appFlagsHandle, APP_FLAG_READY, osFlagsWaitAll, osWaitForever);
  for (;;)
  {
    t1 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    t2 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    t3 = 1U; /* Laser3 default ready */

    (void)osMutexAcquire(laserStateMutexHandle, osWaitForever);
    old1 = g_laser.temp1_ready;
    old2 = g_laser.temp2_ready;
    old3 = g_laser.temp3_ready;

    g_laser.temp1_ready = t1;
    g_laser.temp2_ready = t2;
    g_laser.temp3_ready = t3;
    Laser_UpdateEnableLocked();
    (void)osMutexRelease(laserStateMutexHandle);

    if (old1 != t1)
    {
      Log_Printf("Temp state Laser1=%u\r\n", (unsigned)t1);
    }
    if (old2 != t2)
    {
      Log_Printf("Temp state Laser2=%u\r\n", (unsigned)t2);
    }
    if (old3 != t3)
    {
      Log_Printf("Temp state Laser3=%u\r\n", (unsigned)t3);
    }

    osDelay(TEMP_DEBOUNCE_MS);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the UartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  UartLogMsg_t msg;
  (void)argument;

  (void)osEventFlagsWait(appFlagsHandle, APP_FLAG_READY, osFlagsWaitAll, osWaitForever);
  for (;;)
  {
    if (osMessageQueueGet(uartTxQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg.text, (uint16_t)strlen(msg.text), 200U);
    }
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the AdcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  (void)argument;
  (void)osEventFlagsWait(appFlagsHandle, APP_FLAG_READY, osFlagsWaitAll, osWaitForever);

  for (;;)
  {
    (void)osSemaphoreAcquire(adcReqSemHandle, osWaitForever);

    (void)HAL_ADC_Stop_DMA(&hadc1);
    memset(g_adc_dma_buf, 0, sizeof(g_adc_dma_buf));
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buf, ADC_DMA_CHANNELS) == HAL_OK)
    {
      if (osSemaphoreAcquire(adcCpltSemHandle, 300U) == osOK)
      {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        (void)osMutexAcquire(adcDataMutexHandle, osWaitForever);
        memcpy(g_adc.raw, g_adc_dma_buf, sizeof(g_adc.raw));
        (void)osMutexRelease(adcDataMutexHandle);
      }
      else
      {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        Log_Printf("Error: ADC sample timeout\r\n");
      }
    }
    else
    {
      Log_Printf("Error: ADC start failed\r\n");
    }

    (void)osSemaphoreRelease(adcDoneSemHandle);
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void Log_Printf(const char *fmt, ...)
{
  UartLogMsg_t msg;
  va_list args;

  memset(&msg, 0, sizeof(msg));
  va_start(args, fmt);
  (void)vsnprintf(msg.text, sizeof(msg.text), fmt, args);
  va_end(args);

  if ((osKernelGetState() == osKernelRunning) && (uartTxQueueHandle != NULL))
  {
    if (osMessageQueuePut(uartTxQueueHandle, &msg, 0U, 0U) == osOK)
    {
      return;
    }
  }
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg.text, (uint16_t)strlen(msg.text), 200U);
}

static void Hardware_InitSafe(void)
{
  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  (void)HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  (void)HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);
  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0U);

  (void)osMutexAcquire(i2cMutexHandle, osWaitForever);
  (void)MCP4725_SetVoltage(&hi2c1, 0U);
  (void)osMutexRelease(i2cMutexHandle);

  (void)osMutexAcquire(laserStateMutexHandle, osWaitForever);
  g_laser.temp1_ready = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  g_laser.temp2_ready = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
  g_laser.temp3_ready = 1U;
  Laser_UpdateEnableLocked();
  (void)osMutexRelease(laserStateMutexHandle);

  Log_Printf("System init done, waiting cmd\r\n");
}

static void Laser_UpdateEnableLocked(void)
{
  g_laser.en1 = g_laser.temp1_ready;
  g_laser.en2 = (uint8_t)(g_laser.temp1_ready && g_laser.temp2_ready);
  g_laser.en3 = (uint8_t)(g_laser.temp1_ready && g_laser.temp2_ready && g_laser.temp3_ready);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, g_laser.en1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, g_laser.en2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, g_laser.en3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool Laser_CommandAllowedLocked(uint8_t cmd)
{
  if ((cmd >= (uint8_t)'0') && (cmd <= (uint8_t)'3'))
  {
    return (g_laser.temp1_ready != 0U);
  }
  if ((cmd >= (uint8_t)'4') && (cmd <= (uint8_t)'7'))
  {
    return (g_laser.temp1_ready && g_laser.temp2_ready) != 0U;
  }
  if ((cmd == (uint8_t)'8') || (cmd == (uint8_t)'9'))
  {
    return (g_laser.temp1_ready && g_laser.temp2_ready && g_laser.temp3_ready) != 0U;
  }
  return false;
}

static void Adc_RequestAndWait(void)
{
  (void)osSemaphoreRelease(adcReqSemHandle);
  (void)osSemaphoreAcquire(adcDoneSemHandle, 500U);
}

static float Laser3_ADCtoVoltage(uint16_t adc_value)
{
  float adc_voltage = (float)adc_value * 3.3f / 4096.0f;
  return LASER3_VOLTAGE_MIN +
         (adc_voltage - LASER3_ADC_VOLTAGE_MIN) *
         (LASER3_VOLTAGE_MAX - LASER3_VOLTAGE_MIN) /
         (LASER3_ADC_VOLTAGE_MAX - LASER3_ADC_VOLTAGE_MIN);
}

static float Laser3_ADCtoCurrent(uint16_t adc_value)
{
  float adc_voltage = (float)adc_value * 3.3f / 4096.0f;
  return LASER3_CURRENT_MIN +
         (adc_voltage - LASER3_ADC_CURRENT_MIN) *
         (LASER3_CURRENT_MAX - LASER3_CURRENT_MIN) /
         (LASER3_ADC_CURRENT_MAX - LASER3_ADC_CURRENT_MIN);
}

static void Laser_ProcessCommand(uint8_t cmd)
{
  AdcSnapshot_t snap;
  uint32_t d1;
  uint32_t d2;
  uint16_t d3;
  float input_v, input_i, out_v, out_i;

  (void)osMutexAcquire(laserStateMutexHandle, osWaitForever);
  if (!Laser_CommandAllowedLocked(cmd))
  {
    if ((cmd >= (uint8_t)'0') && (cmd <= (uint8_t)'3'))
    {
      Log_Printf("Reject: Laser1 temp not ready\r\n");
    }
    else if ((cmd >= (uint8_t)'4') && (cmd <= (uint8_t)'7'))
    {
      Log_Printf("Reject: Laser1/Laser2 not ready\r\n");
    }
    else
    {
      Log_Printf("Reject: Laser1/Laser2/Laser3 not ready\r\n");
    }
    (void)osMutexRelease(laserStateMutexHandle);
    return;
  }

  switch (cmd)
  {
    case '0':
      g_laser.dac1 = (g_laser.dac1 >= LASER1_BIG_STEP) ? (g_laser.dac1 - LASER1_BIG_STEP) : 0U;
      break;
    case '1':
      g_laser.dac1 = SAT_ADD_U32(g_laser.dac1, LASER1_BIG_STEP, LASER12_DAC_MAX);
      break;
    case '2':
      g_laser.dac1 = (g_laser.dac1 >= LASER1_SMALL_STEP) ? (g_laser.dac1 - LASER1_SMALL_STEP) : 0U;
      break;
    case '3':
      g_laser.dac1 = SAT_ADD_U32(g_laser.dac1, LASER1_SMALL_STEP, LASER12_DAC_MAX);
      break;
    case '4':
      g_laser.dac2 = SAT_ADD_U32(g_laser.dac2, LASER2_BIG_STEP, LASER12_DAC_MAX);
      break;
    case '5':
      g_laser.dac2 = (g_laser.dac2 >= LASER2_BIG_STEP) ? (g_laser.dac2 - LASER2_BIG_STEP) : 0U;
      break;
    case '6':
      g_laser.dac2 = SAT_ADD_U32(g_laser.dac2, LASER2_SMALL_STEP, LASER12_DAC_MAX);
      break;
    case '7':
      g_laser.dac2 = (g_laser.dac2 >= LASER2_SMALL_STEP) ? (g_laser.dac2 - LASER2_SMALL_STEP) : 0U;
      break;
    case '8':
      g_laser.dac3 = (uint16_t)SAT_ADD_U32(g_laser.dac3, LASER3_STEP, LASER3_DAC_MAX);
      break;
    case '9':
      g_laser.dac3 = (g_laser.dac3 >= LASER3_STEP) ? (g_laser.dac3 - LASER3_STEP) : 0U;
      break;
    default:
      (void)osMutexRelease(laserStateMutexHandle);
      return;
  }

  d1 = g_laser.dac1;
  d2 = g_laser.dac2;
  d3 = g_laser.dac3;
  (void)osMutexRelease(laserStateMutexHandle);

  if ((cmd >= (uint8_t)'0') && (cmd <= (uint8_t)'3'))
  {
    (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, d1);
    osDelay(100U);
  }
  else if ((cmd >= (uint8_t)'4') && (cmd <= (uint8_t)'7'))
  {
    (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, d2);
    osDelay(100U);
  }
  else
  {
    (void)osMutexAcquire(i2cMutexHandle, osWaitForever);
    (void)MCP4725_SetVoltage(&hi2c1, d3);
    (void)osMutexRelease(i2cMutexHandle);
    osDelay(LASER3_SETTLE_DELAY_MS);
  }

  Adc_RequestAndWait();
  (void)osMutexAcquire(adcDataMutexHandle, osWaitForever);
  memcpy(&snap, &g_adc, sizeof(snap));
  (void)osMutexRelease(adcDataMutexHandle);

  if ((cmd >= (uint8_t)'0') && (cmd <= (uint8_t)'3'))
  {
    input_v = (float)d1 * 3.3f / 4096.0f;
    input_i = input_v / 2.5f;
    out_v = (float)snap.raw[0] * 3.3f / 4096.0f;
    out_i = out_v / 2.5f;
    Log_Printf("Laser1: DAC=%lu, InI=%.3fA, InV=%.3fV, OutI=%.3fA, OutV=%.3fV\r\n",
               d1, input_i, input_v, out_i, out_v);
  }
  else if ((cmd >= (uint8_t)'4') && (cmd <= (uint8_t)'7'))
  {
    input_v = (float)d2 * 3.3f / 4096.0f;
    input_i = input_v / 2.5f;
    out_v = (float)snap.raw[1] * 3.3f / 4096.0f;
    out_i = out_v / 2.5f;
    Log_Printf("Laser2: DAC=%lu, InI=%.3fA, InV=%.3fV, OutI=%.3fA, OutV=%.3fV\r\n",
               d2, input_i, input_v, out_i, out_v);
  }
  else
  {
    input_v = (float)d3 * 3.3f / 4096.0f;
    input_i = input_v * 6.13f + 0.8f;
    out_v = Laser3_ADCtoVoltage(snap.raw[2]);
    out_i = Laser3_ADCtoCurrent(snap.raw[3]);
    Log_Printf("Laser3: DAC=%u, InI=%.3fA, InV=%.3fV, OutI=%.3fA, OutV=%.3fV\r\n",
               d3, input_i, input_v, out_i, out_v);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) && (adcCpltSemHandle != NULL))
  {
    (void)osSemaphoreRelease(adcCpltSemHandle);
  }
}
/* USER CODE END Application */
