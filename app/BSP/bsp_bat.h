#ifndef __BSP_BAT_H
#define __BSP_BAT_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * 车身电池电压检测：PA5 = ADC1_IN5
 * 分压电路：电池+ -> 180K -> PA5 -> 10K -> GND（分压比 19 倍）
 * 48V/13S 锂电：满电 54.6V -> ADC 脚 2.87V，放空 39V -> 2.05V
 */

#define BAT_DIV_RATIO     19      /* 分压比 (R1+R2)/R2 */

/* 13S 三元锂阈值（mV） */
#define BAT_VOLT_FULL     54600
#define BAT_VOLT_EMPTY    39000
#define BAT_VOLT_WARN     42000   /* 低电告警 */
#define BAT_VOLT_CRIT     40000   /* 严重低电：强制停车 */

void     BSP_Bat_Init(void);
void     Bat_Update(void);          /* 周期调用（20ms），内部完成滤波和百分比换算 */
uint32_t Bat_GetVoltageMv(void);    /* 当前电池电压 mV */
uint8_t  Bat_GetPercent(void);      /* 电量百分比 0~100（电压查表法估算） */

#endif /* __BSP_BAT_H */
