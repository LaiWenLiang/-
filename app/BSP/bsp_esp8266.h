#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include "bsp.h"

/***************************************************************************************************
 * ESP-01S (ESP8266) 驱动：UART5 透传 + AT 指令
 * 接线：PC12(TX) -> 模块RXD   PD2(RX) -> 模块TXD   PD0 -> RST（可选硬复位）
 *       EN(CH_PD) 硬件接 3.3V    注意 3.3V 供电必须能输出 500mA 以上！
 ***************************************************************************************************/

void    ESP8266_Init(void);

/* 发一条 AT 指令（自动加 \r\n），在 timeout_ms 内等到 ack 字符串返回 1，超时返回 0 */
uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout_ms);

/* 底层接口：发原始字符串 / 发原始数据 */
void    ESP8266_SendStr(const char *str);
void    ESP8266_SendRaw(const uint8_t *data, uint16_t len);

/* 在串口数据流里等某个字符串出现（找到返回 1），用于等 "OK" ">" "CLOSED" 等 */
uint8_t ESP8266_WaitStr(const char *str, uint32_t timeout_ms);

/* 从接收缓冲读一个字节，timeout_ms 内没有数据返回 0 */
uint8_t ESP8266_ReadByte(uint8_t *ch, uint32_t timeout_ms);

/* 清空接收缓冲 */
void    ESP8266_FlushRx(void);

#endif /* __BSP_ESP8266_H */
