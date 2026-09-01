#ifndef __BSP_W25Q32_H
#define __BSP_W25Q32_H

#include "stm32f4xx.h"
#include <stdint.h>

/***************************************************************************************************
 * W25Q32（4MB SPI Flash）驱动
 * 接线：SPI2  PB13(SCK) / PB14(MISO) / PB15(MOSI)，片选 PB12(CS)
 * 特点：一次最多写 256 字节（1页），写前必须先擦除，擦除最小单位 4KB（1扇区）
 ***************************************************************************************************/

void     W25Q32_Init(void);
void     W25Q32_ReadID(uint8_t *mf_id, uint16_t *dev_id);
void     W25Q32_Read(uint32_t addr, uint8_t *buf, uint32_t len);
void     W25Q32_Write(uint32_t addr, const uint8_t *buf, uint32_t len);  /* 自动跨页 + 等忙 */
void     W25Q32_EraseSector(uint32_t addr);                              /* 擦除 addr 所在的 4KB 扇区 */
void     W25Q32_EraseRange(uint32_t addr, uint32_t len);                 /* 擦除覆盖 [addr, addr+len) 的所有扇区 */

#endif /* __BSP_W25Q32_H */
