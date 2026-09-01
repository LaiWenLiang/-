#include "stm32f4xx.h"
#include "ota_config.h"
#include "bsp_w25q32.h"
#include "bsp_w24c02.h"
#include "bsp_flash.h"
#include "bsp_crc32.h"
#include "boot_jump.h"
#include <string.h>
#include <stdio.h>

/***************************************************************************************************
 * Bootloader 主流程（上电后最先运行）：
 *
 *   1. 读 EEPROM 启动标志
 *   2. 标志 = UPDATE   -> 把当前 APP 备份到 W25Q32，再把新固件刷入 APP 区，标志改 PENDING
 *   3. 标志 = PENDING  -> 上次升级的新 APP 没通过"启动确认"，从 W25Q32 备份区回滚旧版本
 *   4. 其它情况        -> 直接跳转 APP
 ***************************************************************************************************/

/* ---------------- 日志串口（USART1 PA9/PA10，仅调试用） ---------------- */
static void Boot_UartInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

int fputc(int ch, FILE *f)
{
    (void)f;
    while ((USART1->SR & USART_FLAG_TC) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/* ---------------- EEPROM 启动标志读写 ---------------- */
static uint8_t Boot_ReadFlag(void)
{
    uint8_t data[3];

    W24C02_ReadBytes(EEPROM_FLAG_ADDR, data, 3);

    /* 密钥不对（第一次烧录或 EEPROM 被擦过）：重置为"无更新" */
    if (data[1] != EEPROM_KEY_H || data[2] != EEPROM_KEY_L)
    {
        data[0] = OTA_FLAG_NO_UPDATE;
        data[1] = EEPROM_KEY_H;
        data[2] = EEPROM_KEY_L;
        W24C02_WriteBytes(EEPROM_FLAG_ADDR, data, 3);
    }
    return data[0];
}

static void Boot_WriteFlag(uint8_t flag)
{
    uint8_t data[3];

    data[0] = flag;
    data[1] = EEPROM_KEY_H;
    data[2] = EEPROM_KEY_L;
    W24C02_WriteBytes(EEPROM_FLAG_ADDR, data, 3);
}

/* ---------------- 搬运用的小缓冲 ---------------- */
static uint8_t s_buf[256];

/* 把当前 APP 运行区（固定 256KB）备份到 W25Q32 备份区 */
static void Boot_BackupApp(void)
{
    uint32_t i;

    printf("backup old app to w25q32...\n");
    W25Q32_EraseRange(W25Q32_BACKUP_ADDR, OTA_APP_MAX_SIZE);

    for (i = 0; i < OTA_APP_MAX_SIZE; i += sizeof(s_buf))
    {
        Flash_Read(OTA_APP_ADDR + i, s_buf, sizeof(s_buf));
        W25Q32_Write(W25Q32_BACKUP_ADDR + i, s_buf, sizeof(s_buf));
    }
}

/* 把 W25Q32 暂存区的新固件刷入 APP 运行区并回读校验，返回 1=成功 0=失败 */
static uint8_t Boot_WriteNewApp(uint32_t app_size, uint32_t expect_crc)
{
    uint32_t i;
    uint32_t crc;

    Flash_EraseAppRegion();

    for (i = 0; i < app_size; i += sizeof(s_buf))
    {
        uint32_t once = app_size - i;
        if (once > sizeof(s_buf))
        {
            once = sizeof(s_buf);
        }
        W25Q32_Read(W25Q32_STAGE_ADDR + i, s_buf, once);
        Flash_Write(OTA_APP_ADDR + i, s_buf, once);
    }

    /* 回读校验（内部 Flash 可以直接当内存读） */
    crc = CRC32_Calc((const uint8_t *)OTA_APP_ADDR, app_size);
    printf("flash crc: calc=0x%08lX expect=0x%08lX\n", crc, expect_crc);
    return (crc == expect_crc) ? 1 : 0;
}

/* 校验 W25Q32 暂存区里的新固件 CRC，返回 1=通过 */
static uint8_t Boot_CheckStageCrc(uint32_t app_size, uint32_t expect_crc)
{
    uint32_t i;
    uint32_t crc = 0xFFFFFFFF;

    for (i = 0; i < app_size; i += sizeof(s_buf))
    {
        uint32_t once = app_size - i;
        if (once > sizeof(s_buf))
        {
            once = sizeof(s_buf);
        }
        W25Q32_Read(W25Q32_STAGE_ADDR + i, s_buf, once);
        crc = CRC32_Append(crc, s_buf, once);
    }
    crc = ~crc;

    printf("stage crc: calc=0x%08lX expect=0x%08lX\n", crc, expect_crc);
    return (crc == expect_crc) ? 1 : 0;
}

/* 回滚函数在后面定义，先声明 */
static void Boot_DoRollback(void);

/* 执行更新：备份旧 APP -> 刷入新固件 -> 标志改 PENDING */
static void Boot_DoUpdate(void)
{
    uint8_t  meta[8];
    uint32_t app_size;
    uint32_t app_crc;

    /* 读元数据：前 4 字节大小，后 4 字节 CRC32，都是低字节在前 */
    W25Q32_Read(W25Q32_META_ADDR, meta, 8);
    app_size = (uint32_t)meta[0] | ((uint32_t)meta[1] << 8) |
               ((uint32_t)meta[2] << 16) | ((uint32_t)meta[3] << 24);
    app_crc  = (uint32_t)meta[4] | ((uint32_t)meta[5] << 8) |
               ((uint32_t)meta[6] << 16) | ((uint32_t)meta[7] << 24);

    printf("new app size=%lu crc=0x%08lX\n", app_size, app_crc);

    /* 大小合法性检查 */
    if (app_size < 512 || app_size > OTA_APP_MAX_SIZE)
    {
        printf("bad app size, cancel update\n");
        Boot_WriteFlag(OTA_FLAG_NO_UPDATE);
        return;
    }

    /* 暂存区 CRC 检查，不过就不动旧 APP（防变砖） */
    if (Boot_CheckStageCrc(app_size, app_crc) == 0)
    {
        printf("stage crc fail, cancel update\n");
        Boot_WriteFlag(OTA_FLAG_NO_UPDATE);
        return;
    }

    Boot_BackupApp();
    if (Boot_WriteNewApp(app_size, app_crc) == 0)
    {
        /* 刷写失败：旧 APP 已被擦除，但备份还在，直接回滚补救 */
        printf("write flash fail, rollback now\n");
        Boot_DoRollback();
        return;
    }

    /* 等新 APP 首次启动自我确认，确认不了下次就回滚 */
    Boot_WriteFlag(OTA_FLAG_PENDING);
    printf("update ok, wait app confirm\n");
}

/* 回滚：把 W25Q32 备份区的旧 APP 刷回 APP 运行区 */
static void Boot_DoRollback(void)
{
    uint32_t i;

    printf("rollback to old app...\n");
    Flash_EraseAppRegion();

    for (i = 0; i < OTA_APP_MAX_SIZE; i += sizeof(s_buf))
    {
        W25Q32_Read(W25Q32_BACKUP_ADDR + i, s_buf, sizeof(s_buf));
        Flash_Write(OTA_APP_ADDR + i, s_buf, sizeof(s_buf));
    }

    Boot_WriteFlag(OTA_FLAG_CONFIRMED);
    printf("rollback done\n");
}

int main(void)
{
    uint8_t flag;

    Boot_UartInit();
    W25Q32_Init();
    W24C02_Init();

    printf("\nbootloader start\n");

    flag = Boot_ReadFlag();
    printf("boot flag = 0x%02X\n", flag);

    if (flag == OTA_FLAG_UPDATE)
    {
        Boot_DoUpdate();
    }
    else if (flag == OTA_FLAG_PENDING)
    {
        Boot_DoRollback();
    }

    printf("jump to app\n");
    if (Boot_JumpToApp(OTA_APP_ADDR) == 0)
    {
        /* APP 不存在或损坏：停在这里等看门狗/重新烧录 */
        printf("app invalid, stop here\n");
        while (1)
        {
        }
    }

    while (1)
    {
    }
}
