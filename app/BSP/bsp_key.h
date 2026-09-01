#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * 按键：低电平触发（按下接地，内部上拉）
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

#endif /* __BSP_KEY_H */
