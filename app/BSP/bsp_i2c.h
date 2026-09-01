#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "bsp.h"

/* TOF250 专用：硬件 I2C2 (PB10 SCL / PB11 SDA)，带超时保护，配合互斥锁使用 */
#define BSP_I2C_TIMEOUT_MS   10

void    BSP_I2C2_Init(void);
int8_t  BSP_I2C2_Read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);
int8_t  BSP_I2C2_Write(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, uint16_t len);

#endif /* __BSP_I2C_H */
