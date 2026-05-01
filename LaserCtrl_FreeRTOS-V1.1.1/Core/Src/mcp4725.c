#include "mcp4725.h"
#include "i2c.h"  // 确保包含I2C头文件

/**
  * @brief  设置MCP4725输出电压
  * @param  hi2c: I2C句柄指针
  * @param  voltage: 12位数字值 (0-4095)
  * @retval HAL状态
  */
HAL_StatusTypeDef MCP4725_SetVoltage(I2C_HandleTypeDef *hi2c, uint16_t voltage)
{
  uint8_t data[3];
  
  // 限制输入范围
  voltage = (voltage > 4095) ? 4095 : voltage;
  
  // 数据格式: [控制字节] [高8位] [低4位(左移4位)]
  data[0] = 0x40;  // Fast Write模式 (C2=0, C1=1, C0=0)
  data[1] = voltage >> 4;          // 取高8位 (D11-D4)
  data[2] = (voltage & 0x0F) << 4; // 取低4位左移 (D3-D0)
  
  // 发送I2C数据
  return HAL_I2C_Master_Transmit(hi2c, MCP4725_ADDR, data, 3, 100);
}

/**
  * @brief  读取MCP4725寄存器
  * @param  hi2c: I2C句柄指针
  * @param  data: 存储读取数据的缓冲区(5字节)
  * @retval HAL状态
  */
HAL_StatusTypeDef MCP4725_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t *data)
{
  return HAL_I2C_Master_Receive(hi2c, MCP4725_ADDR | 0x01, data, 5, 100);
}
