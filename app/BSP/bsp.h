#ifndef __BSP_H
#define __BSP_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/***************************************************************************************************
 * 默认引脚分配（可按实际接线修改）
 *  UWB 基站 : USART2  PA2(TX) / PA3(RX)   DMA1 Stream5 + 空闲中断
 *  电机PWM  : TIM3 CH3/CH4 -> PB0/PB1 (电机A)   TIM3 CH1/CH2 -> PA6/PA7 (电机B)
 *             TIM4 CH1~CH4 -> PB6/PB7/PB8/PB9 (电机C/D)
 *  TOF250×4: I2C2    PB10(SCL) / PB11(SDA)
 *  OLED    : 软件I2C PE0(SCL) / PE1(SDA)
 *  语音模块: USART3  PC10(TX) / PC11(RX)
 *  LED/蜂鸣器: PF9(LED1) / PF10(LED2) / PF8(BEEP)
 *  W25Q32  : SPI2  PB13(SCK) / PB14(MISO) / PB15(MOSI) / PB12(CS)   OTA 固件暂存+备份
 *  W24C02  : 软件I2C PF6(SCL) / PF7(SDA)                            OTA 启动标志
 *  ESP8266 : UART5 PC12(TX) / PD2(RX)，RST=PD0，EN 接 3.3V          OneNET 远程升级
 ***************************************************************************************************/

void BSP_Init(void);

#endif /* __BSP_H */
