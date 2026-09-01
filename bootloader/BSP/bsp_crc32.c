#include "bsp_crc32.h"

/***************************************************************************************************
 * CRC32（逐位计算版，不用查表，代码最简单；升级场景对速度不敏感）
 * 多项式 0xEDB88320，初值 0xFFFFFFFF，最终结果取反 —— 与 Python zlib.crc32() 一致
 ***************************************************************************************************/
uint32_t CRC32_Append(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t  bit;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xEDB88320;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t CRC32_Calc(const uint8_t *data, uint32_t len)
{
    return ~CRC32_Append(0xFFFFFFFF, data, len);
}
