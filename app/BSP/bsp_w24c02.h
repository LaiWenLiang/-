#ifndef __BSP_W24C02_H
#define __BSP_W24C02_H

#include "stm32f4xx.h"
#include <stdint.h>

/***************************************************************************************************
 * W24C02（2Kbit = 256字节 EEPROM）驱动，软件模拟 I2C
 * 接线：PF6(SCL) / PF7(SDA)，SDA 需外部上拉（模块一般自带）
 * 用途：只存 3 字节的 OTA 启动标志，读写量极小，软件 I2C 足够
 ***************************************************************************************************/

void    W24C02_Init(void);
void    W24C02_WriteBytes(uint8_t addr, const uint8_t *data, uint8_t len);
void    W24C02_ReadBytes(uint8_t addr, uint8_t *data, uint8_t len);

#endif /* __BSP_W24C02_H */
