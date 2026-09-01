#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "stm32f4xx.h"
#include <stdint.h>

/***************************************************************************************************
 * STM32F407 内部 Flash 操作（BOOTLOADER 与 APP 共用）
 * 注意：F4 是按"扇区"擦除的（不像 F1 按页），本工程 APP 区 = Sector 5 + Sector 6
 ***************************************************************************************************/

/* 擦除 APP 运行区（Sector 5 和 Sector 6） */
void    Flash_EraseAppRegion(void);

/* 从内部 Flash 读数据（其实就是内存拷贝） */
void    Flash_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/* 写数据到内部 Flash（按字写入，地址必须 4 字节对齐，写前要先擦除） */
void    Flash_Write(uint32_t addr, const uint8_t *buf, uint32_t len);

#endif /* __BSP_FLASH_H */
