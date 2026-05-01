#ifndef __MCP4725_H
#define __MCP4725_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"  // HAL库支持

// MCP4725 I2C地址（7位地址左移1位）
#define MCP4725_ADDR 0xC0  // 0x62 << 1

// 函数声明
HAL_StatusTypeDef MCP4725_SetVoltage(I2C_HandleTypeDef *hi2c, uint16_t voltage);
HAL_StatusTypeDef MCP4725_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __MCP4725_H */

