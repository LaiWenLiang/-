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
 *  无刷电机 : CAN1   PB8/PB9 (500K)，驱动器 ID 1=左 / 2=右
 *  TOF250×4: I2C2    PB10(SCL) / PB11(SDA)
 *  OLED    : 软件I2C PE0(SCL) / PE1(SDA)
 *  语音模块: USART3  PC10(TX) / PC11(RX)
 *  LED/蜂鸣器: PF9(LED1) / PF10(LED2) / PF8(BEEP)
 *  按键    : PA0(KEY1) / PA1(KEY2) / PA4(KEY3)，低电平触发
 *  电池检测: ADC1_IN5 PA5（180K+10K 分压）
 *  RS485 DE: PD3（USART1 复用蓝牙/RS485）
 *  W25Q32  : SPI2  PB13(SCK) / PB14(MISO) / PB15(MOSI) / PB12(CS)   OTA 固件暂存+备份
 *  W24C02  : 软件I2C PF6(SCL) / PF7(SDA)                            OTA 启动标志
 *  ESP8266 : UART5 PC12(TX) / PD2(RX)，RST=PD0，EN 接 3.3V          OneNET 远程升级
 ***************************************************************************************************/

void BSP_Init(void);

#endif /* __BSP_H */
