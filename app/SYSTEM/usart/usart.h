#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>
#include <stdio.h>

/***************************************************************************************************
 * 全工程串口统一入口：
 *   USART1 (PA9/PA10) : 调试口，蓝牙(VOFA文本)/RS485(固件下载)二选一复用，KEY3 切换
 *                       RS485 方向脚 DE = PD3（高=发送，低=接收）
 *   USART2 (PA2/PA3)  : UWB 基站（DMA 循环接收 + 空闲中断，整帧入队）
 *   USART3 (PC10/PC11): 语音播报模块（只发）
 ***************************************************************************************************/

#include "ota_config.h"   /* OtaFrame_t */

#define UWB_RX_BUF_SIZE   256

/* USART1 工作模式 */
#define UART1_MODE_BT    0   /* 蓝牙：VOFA 文本行 */
#define UART1_MODE_485   1   /* RS485：OTA 升级帧(0x55 0xAA 头) */

extern uint8_t g_uart1_mode;   /* 当前 USART1 模式，由 KEY3 切换 */

/* UWB 帧容器：DMA 空闲中断把一整帧拷入该结构投递到队列 */
typedef struct
{
    uint16_t len;
    uint8_t  buf[UWB_RX_BUF_SIZE];
} UWB_Frame_t;

/* USART1 调试口 */
void     Debug_UART_Init(uint32_t baudrate);
uint16_t Debug_UART_ReadLine(char *buf, uint16_t maxlen);      /* 取一整行，无数据返回0 */
uint16_t Debug_UART_Write(const uint8_t *data, uint16_t len);  /* DMA 发送一帧 */
uint16_t Debug_Printf(const char *fmt, ...);                   /* 格式化打印（DMA 发送） */
void     Debug_UART_SetOtaQueue(QueueHandle_t rx_queue);       /* 注册 RS485 OTA 帧接收队列 */

/* USART2 UWB */
void     UWB_UART_Init(uint32_t baudrate, QueueHandle_t rx_queue);
uint32_t UWB_UART_LastRxTick(void);

/* USART3 语音 */
void Speaker_UART_Init(uint32_t baudrate);
void Speaker_UART_Send(const uint8_t *data, uint16_t len);

#endif /* __USART_H */
