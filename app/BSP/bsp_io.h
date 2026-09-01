#ifndef __BSP_IO_H
#define __BSP_IO_H

#include "stm32f4xx.h"
#include <stdint.h>

/***************************************************************************************************
 * 简单 IO 类外设合一：LED/蜂鸣器/看门狗 + 按键 + 电池电压检测
 ***************************************************************************************************/

/* ---------- LED / 蜂鸣器（PF9/PF10/PF8） ---------- */
#define LED1_PIN    GPIO_Pin_9
#define LED2_PIN    GPIO_Pin_10
#define BEEP_PIN    GPIO_Pin_8
#define LED_GPIO    GPIOF
#define BEEP_GPIO   GPIOF

#define ON  0
#define OFF 1

#define LED1(a)  do { if(a) GPIO_SetBits(LED_GPIO, LED1_PIN);  else GPIO_ResetBits(LED_GPIO, LED1_PIN);  } while (0)
#define LED2(a)  do { if(a) GPIO_SetBits(LED_GPIO, LED2_PIN);  else GPIO_ResetBits(LED_GPIO, LED2_PIN);  } while (0)
#define BEEP(a)  do { if(a) GPIO_SetBits(BEEP_GPIO, BEEP_PIN); else GPIO_ResetBits(BEEP_GPIO, BEEP_PIN); } while (0)

void BSP_Periph_Init(void);
void BSP_Iwdg_Init(void);
void BSP_Iwdg_Feed(void);

/* ---------- 按键：低电平触发（按下接地，内部上拉） ----------
 *  KEY1 = PA0   升级提示时 = 接受下载
 *  KEY2 = PA1   升级提示时 = 拒绝下载
 *  KEY3 = PA4   平时 = 切换 USART1 蓝牙/RS485 模式
 */
#define KEY1_PIN   GPIO_Pin_0
#define KEY2_PIN   GPIO_Pin_1
#define KEY3_PIN   GPIO_Pin_4
#define KEY_GPIO   GPIOA

#define KEY_NONE   0
#define KEY_1      1
#define KEY_2      2
#define KEY_3      3

void    BSP_Key_Init(void);
uint8_t Key_Scan(void);   /* 返回按下的键值，无按键返回 KEY_NONE，需周期调用(<=50ms) */

/* ---------- 电池电压检测：PA5 = ADC1_IN5 ----------
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

#endif /* __BSP_IO_H */
