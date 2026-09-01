#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include "bsp.h"

/* 软件 I2C 引脚：PE0=SCL  PE1=SDA */
void BSP_OLED_Init(void);
void BSP_OLED_Clear(void);
void BSP_OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void BSP_OLED_DisplayOnOff(uint8_t on);   /* 1=开显示 0=关显示（省电） */

#endif /* __BSP_OLED_H */
