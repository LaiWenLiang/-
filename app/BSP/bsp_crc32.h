#ifndef __BSP_CRC32_H
#define __BSP_CRC32_H

#include <stdint.h>

/* 一次性计算 CRC32（多项式 0xEDB88320，初值 0xFFFFFFFF，结果取反，同 zlib.crc32） */
uint32_t CRC32_Calc(const uint8_t *data, uint32_t len);

/* 分段计算：crc 第一次传 0xFFFFFFFF，算完所有段后对结果取反(~) 就是最终 CRC32 */
uint32_t CRC32_Append(uint32_t crc, const uint8_t *data, uint32_t len);

/* CRC16-Modbus（多项式 0xA001，初值 0xFFFF），RS485 升级帧校验用 */
uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len);

#endif /* __BSP_CRC32_H */
