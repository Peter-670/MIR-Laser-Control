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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp4725.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_DEBOUNCE_MS 200 // 防抖动时间
#define CHANNELS 4
// Laser3 ADC转换系数
#define LASER3_VOLTAGE_MIN 2.0f    // 最小输出电压
#define LASER3_VOLTAGE_MAX 3.3f    // 最大输出电压
#define LASER3_ADC_VOLTAGE_MIN 0.0f // PA6最小输入电压
#define LASER3_ADC_VOLTAGE_MAX 3.26f // PA6最大输入电压
#define LASER3_CURRENT_MIN 0.8f    // 最小输出电流
#define LASER3_CURRENT_MAX 10.0f   // 最大输出电流
#define LASER3_ADC_CURRENT_MIN 0.0f // PA7最小输入电压
#define LASER3_ADC_CURRENT_MAX 2.71f // PA7最大输入电压
#define LASER3_SETTLE_DELAY 200    // Laser3电压稳定延迟(ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_Value[CHANNELS];

// Laser1 独立参数
uint32_t DAC_Value1 = 0;
float Im_value1, Ic_value1, Input_Vcc1, Output_Vcc1;
uint8_t temp_status_prev1 = 0; // 0:未就绪, 1:已就绪
uint32_t last_temp_check1 = 0;

// Laser2 独立参数
uint32_t DAC_Value2 = 0;
float Im_value2, Ic_value2, Input_Vcc2, Output_Vcc2;
uint8_t temp_status_prev2 = 0;
uint32_t last_temp_check2 = 0;

// Laser3 独立参数
uint16_t DAC_Value3 = 0;
float Im_value3, Ic_value3, Input_Vcc3, Output_Vcc3;
float Output_Current3; // Laser3专用输出电流
uint8_t temp_status_prev3 = 1; // 修改：默认设置为1（温度就绪）
uint32_t last_temp_check3 = 0;

// 全局串口指令（单次接收，统一分配）
uint8_t uart_cmd = 0; // 0表示无指令

// 激光器使能状态
uint8_t laser1_enabled = 0;
uint8_t laser2_enabled = 0;
uint8_t laser3_enabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 优先级检查函数
uint8_t CheckLaser1Priority(void);
uint8_t CheckLaser2Priority(void);
uint8_t CheckLaser3Priority(void);
// 更新激光器使能状态
void UpdateLaserEnableStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 激光3电压转换函数：将PA6的ADC值转换为实际输出电压
float Laser3_ADCtoVoltage(uint16_t adc_value) {
    float adc_voltage = adc_value * 3.3f / 4096.0f;
    // 从ADC电压映射到实际输出电压 (0-3.26V -> 2V-3.3V)
    return LASER3_VOLTAGE_MIN + (adc_voltage - LASER3_ADC_VOLTAGE_MIN) * 
           (LASER3_VOLTAGE_MAX - LASER3_VOLTAGE_MIN) / 
           (LASER3_ADC_VOLTAGE_MAX - LASER3_ADC_VOLTAGE_MIN);
}

// 激光3电流转换函数：将PA7的ADC值转换为实际输出电流
float Laser3_ADCtoCurrent(uint16_t adc_value) {
    float adc_voltage = adc_value * 3.3f / 4096.0f;
    // 从ADC电压映射到实际输出电流 (0-2.71V -> 0.8-10A)
    return LASER3_CURRENT_MIN + (adc_voltage - LASER3_ADC_CURRENT_MIN) * 
           (LASER3_CURRENT_MAX - LASER3_CURRENT_MIN) / 
           (LASER3_ADC_CURRENT_MAX - LASER3_ADC_CURRENT_MIN);
}

// 强制ADC进行一次新的转换并获取最新值
void UpdateADCValues(void) {
    // 停止当前DMA转换
    HAL_ADC_Stop_DMA(&hadc1);
    // 清除之前的值
    for(int i = 0; i < CHANNELS; i++) {
        ADC_Value[i] = 0;
    }
    // 启动新的DMA转换
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, CHANNELS);
    // 等待转换完成
    HAL_Delay(50);
}

// 检查激光器1优先级（最高优先级，无依赖）
uint8_t CheckLaser1Priority(void) {
    return temp_status_prev1; // 只需要自身温度就绪
}

// 检查激光器2优先级（依赖激光器1）
uint8_t CheckLaser2Priority(void) {
    return (temp_status_prev1 && temp_status_prev2); // 需要激光器1和2都温度就绪
}

// 检查激光器3优先级（依赖激光器1和2）
uint8_t CheckLaser3Priority(void) {
    return (temp_status_prev1 && temp_status_prev2 && temp_status_prev3); // 需要所有激光器温度就绪
}

// 更新激光器使能状态
void UpdateLaserEnableStatus(void) {
    // 更新激光器1使能状态
    laser1_enabled = CheckLaser1Priority();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, laser1_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 更新激光器2使能状态
    laser2_enabled = CheckLaser2Priority();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, laser2_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 更新激光器3使能状态
    laser3_enabled = CheckLaser3Priority();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, laser3_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("欢迎使用MIR LASER激光控制系统，使用说明见手册\r\n");
  
  // 初始化激光器参数
  // Laser1
  Input_Vcc1 = DAC_Value1 * 3.3f / 4096.0f;
  Ic_value1 = Input_Vcc1 / 2.5f;
  
  // Laser2
  Input_Vcc2 = DAC_Value2 * 3.3f / 4096.0f;
  Ic_value2 = Input_Vcc2 / 2.5f;
  
  // Laser3
  Input_Vcc3 = DAC_Value3 * 3.3f / 4096.0f;
  Ic_value3 = Input_Vcc3 * 6.13f;
  
  // 外设初始化
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);    // 启动Laser1的DAC通道
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);    // 启动Laser2的DAC通道
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, CHANNELS); // 启动ADC DMA
  
  // 初始读取一次ADC值
  UpdateADCValues();
  
  printf("系统初始化完成，等待指令...\r\n");
	
  MCP4725_SetVoltage(&hi2c1, 0);  //先给DAC3置零
  printf("Laser3 DAC: 已初始化\r\n");
  
  // 修改：初始化时直接设置激光器3为温度就绪状态
  temp_status_prev3 = 1;
  printf("Laser3: 温度就绪\r\n");
  
  // 初始更新激光器使能状态
  UpdateLaserEnableStatus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /************************** 温度检测（并行逻辑，互不干扰） **************************/
    // Laser1 温度检测
    if (HAL_GetTick() - last_temp_check1 >= TEMP_DEBOUNCE_MS)
    {
      last_temp_check1 = HAL_GetTick();
      uint8_t temp_status1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
      if (temp_status1 != temp_status_prev1)
      {
        temp_status_prev1 = temp_status1;
        if (temp_status1)
          printf("Laser1: 温度就绪，已使能\r\n");
        else
          printf("Laser1: 温度未就绪，已禁用\r\n");
        UpdateLaserEnableStatus(); // 更新所有激光器使能状态
      }
    }

    // Laser2 温度检测
    if (HAL_GetTick() - last_temp_check2 >= TEMP_DEBOUNCE_MS)
    {
      last_temp_check2 = HAL_GetTick();
      uint8_t temp_status2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
      if (temp_status2 != temp_status_prev2)
      {
        temp_status_prev2 = temp_status2;
        if (temp_status2)
          printf("Laser2: 温度就绪\r\n");
        else
          printf("Laser2: 温度未就绪\r\n");
        UpdateLaserEnableStatus(); // 更新所有激光器使能状态
      }
    }

    // 修改：跳过Laser3温度检测，直接保持温度就绪状态
    // 原来的温度检测代码已注释掉
    /*
    // Laser3 温度检测
    if (HAL_GetTick() - last_temp_check3 >= TEMP_DEBOUNCE_MS)
    {
      last_temp_check3 = HAL_GetTick();
      uint8_t temp_status3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
      if (temp_status3 != temp_status_prev3)
      {
        temp_status_prev3 = temp_status3;
        if (temp_status3)
          printf("Laser3: 温度就绪\r\n");
        else
          printf("Laser3: 温度未就绪\r\n");
        UpdateLaserEnableStatus(); // 更新所有激光器使能状态
      }
    }
    */

    /************************** 串口接收（单次接收） **************************/
    HAL_StatusTypeDef uart_status = HAL_UART_Receive(&huart2, &uart_cmd, 1, 100);
    // 仅在接收到有效数据时处理指令（HAL_OK表示成功接收1字节）
    if (uart_status != HAL_OK)
    {
      uart_cmd = 0; // 超时或错误时清空指令，避免重复执行
      continue;
    }

    /************************** 指令分配与处理（独立逻辑） **************************/
    // 1. 处理Laser1（指令0-3）
    if (uart_cmd >= '0' && uart_cmd <= '3')
    {
      if (!CheckLaser1Priority())
      {
        printf("错误：Laser1温度未就绪，无法执行指令\r\n");
        uart_cmd = 0;
        continue;
      }

      // 根据指令更新DAC_Value1
      switch(uart_cmd)
      {
        case '0': // 减小31（大步）
          DAC_Value1 = (DAC_Value1 >= 31) ? (DAC_Value1 - 31) : 0;
          break;
        case '1': // 增大31（大步）
          DAC_Value1 = (DAC_Value1 <= 3103) ? (DAC_Value1 + 31) : 3103;
          break;
        case '2': // 减小3（小步）
          DAC_Value1 = (DAC_Value1 >= 3) ? (DAC_Value1 - 3) : 0;
          break;
        case '3': // 增大3（小步）
          DAC_Value1 = (DAC_Value1 <= 3103) ? (DAC_Value1 + 3) : 3103;
          break;
      }

      // 设置DAC值
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Value1);
      
      // 等待稳定并更新ADC
      HAL_Delay(100);
      UpdateADCValues();
      
      // 计算参数
      Input_Vcc1 = DAC_Value1 * 3.3f / 4096.0f;
      Ic_value1 = Input_Vcc1 / 2.5f;
      Im_value1 = ADC_Value[0] * 3.3f / 4096.0f / 2.5f;
      Output_Vcc1 = ADC_Value[0] * 3.3f / 4096.0f;
      
      // 打印信息
      printf("Laser1: DAC=%d, 输入电流=%.3fA, 输入电压=%.3fV, 输出电流=%.3fA, 输出电压=%.3fV\r\n",
             DAC_Value1, Ic_value1, Input_Vcc1, Im_value1, Output_Vcc1);
    }

    // 2. 处理Laser2（指令4-7）
    else if (uart_cmd >= '4' && uart_cmd <= '7')
    {
      if (!CheckLaser2Priority())
      {
        if (!temp_status_prev1)
          printf("错误：Laser1未就绪，无法控制Laser2\r\n");
        else if (!temp_status_prev2)
          printf("错误：Laser2温度未就绪，无法执行指令\r\n");
        uart_cmd = 0;
        continue;
      }

      // 根据指令更新DAC_Value2
      switch(uart_cmd)
      {
        case '4': // 增大31（大步）
          DAC_Value2 = (DAC_Value2 <= 3103) ? (DAC_Value2 + 31) : 3103;
          break;
        case '5': // 减小31（大步）
          DAC_Value2 = (DAC_Value2 >= 31) ? (DAC_Value2 - 31) : 0;
          break;
        case '6': // 增大3（小步）
          DAC_Value2 = (DAC_Value2 <= 3103) ? (DAC_Value2 + 3) : 3103;
          break;
        case '7': // 减小3（小步）
          DAC_Value2 = (DAC_Value2 >= 3) ? (DAC_Value2 - 3) : 0;
          break;
      }

      // 设置DAC值
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_Value2);
      
      // 等待稳定并更新ADC
      HAL_Delay(100);
      UpdateADCValues();
      
      // 计算参数
      Input_Vcc2 = DAC_Value2 * 3.3f / 4096.0f;
      Ic_value2 = Input_Vcc2 / 2.5f;
      Im_value2 = ADC_Value[1] * 3.3f / 4096.0f / 2.5f;
      Output_Vcc2 = ADC_Value[1] * 3.3f / 4096.0f;
      
      // 打印信息
      printf("Laser2: DAC=%d, 输入电流=%.3fA, 输入电压=%.3fV, 输出电流=%.3fA, 输出电压=%.3fV\r\n",
             DAC_Value2, Ic_value2, Input_Vcc2, Im_value2, Output_Vcc2);
    }

    // 3. 处理Laser3（指令8-9）- 核心修改部分
    else if (uart_cmd >= '8' && uart_cmd <= '9')
    {
      if (!CheckLaser3Priority())
      {
        if (!temp_status_prev1)
          printf("错误：Laser1未就绪，无法控制Laser3\r\n");
        else if (!temp_status_prev2)
          printf("错误：Laser2未就绪，无法控制Laser3\r\n");
        // 修改：移除激光器3温度未就绪的错误提示，因为现在总是就绪
        uart_cmd = 0;
        continue;
      }

      // 1. 首先根据指令更新DAC值
      switch(uart_cmd)
      {
        case '8': // 增大20
          DAC_Value3 = (DAC_Value3 <= 1861) ? (DAC_Value3 + 20) : 1861;
          break;
        case '9': // 减小20
          DAC_Value3 = (DAC_Value3 >= 20) ? (DAC_Value3 - 20) : 0;
          break;
      }

      // 2. 应用DAC设置到硬件（先设置）
      MCP4725_SetVoltage(&hi2c1, DAC_Value3);
      
     // 3. 等待电压稳定（延长延迟确保稳定）
     HAL_Delay(LASER3_SETTLE_DELAY);
      
      // 4. 强制更新ADC值，确保获取最新的测量数据
      UpdateADCValues();
      
      // 5. 最后计算并显示参数（后显示）
      Input_Vcc3 = DAC_Value3 * 3.3f / 4096.0f;
      Ic_value3 = Input_Vcc3 * 6.13f;
      
      // 读取ADC数据（PA6:电压, PA7:电流）
      Output_Vcc3 = Laser3_ADCtoVoltage(ADC_Value[2]);  // PA6对应ADC_Value[2]
      Output_Current3 = Laser3_ADCtoCurrent(ADC_Value[3]); // PA7对应ADC_Value[3]
      
      // 打印信息
      printf("Laser3: DAC=%d, 输入电流=%.3fA, 输入电压=%.3fV, 输出电流=%.3fA, 输出电压=%.3fV\r\n",
             DAC_Value3, Ic_value3+0.8, Input_Vcc3, Output_Current3, Output_Vcc3);
    }

    // 无效指令处理
    else
    {
      printf("错误：无效指令（仅支持0-9）\r\n");
    }

    uart_cmd = 0; // 指令处理完成后清空，确保单次执行
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    // 打印ADC原始数据，便于调试
    printf("ADC原始数据：CH0(Laser1)=%d, CH1(Laser2)=%d, CH2(Laser3电压)=%d, CH3(Laser3电流)=%d\r\n",
           ADC_Value[0], ADC_Value[1], ADC_Value[2], ADC_Value[3]);
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

