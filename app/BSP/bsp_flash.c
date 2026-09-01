#include "bsp_flash.h"
#include "ota_config.h"
#include <string.h>

void Flash_EraseAppRegion(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* 电压范围 3.3V -> VoltageRange_3，擦除时间约 1~2 秒/扇区 */
    FLASH_EraseSector(OTA_APP_SECTOR_1, VoltageRange_3);
    FLASH_EraseSector(OTA_APP_SECTOR_2, VoltageRange_3);

    FLASH_Lock();
}

void Flash_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    memcpy(buf, (const void *)addr, len);
}

void Flash_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t word;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (i = 0; i + 4 <= len; i += 4)
    {
        /* 拼成 32 位字，低字节在前 */
        word = (uint32_t)buf[i]
             | ((uint32_t)buf[i + 1] << 8)
             | ((uint32_t)buf[i + 2] << 16)
             | ((uint32_t)buf[i + 3] << 24);
        FLASH_ProgramWord(addr + i, word);
    }

    /* 尾部不足 4 字节的零头（正常固件都是 4 对齐，这里兜底） */
    if (i < len)
    {
        word = 0xFFFFFFFF;
        memcpy(&word, buf + i, len - i);
        FLASH_ProgramWord(addr + i, word);
    }

    FLASH_Lock();
}
