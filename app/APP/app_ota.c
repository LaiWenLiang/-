#include "app_ota.h"
#include "app.h"
#include "ota_config.h"
#include "bsp_w25q32.h"
#include "bsp_w24c02.h"
#include "bsp_crc32.h"
#include "usart.h"
#include <string.h>

/***************************************************************************************************
 * OTA 接收状态机（RS485 通道）：
 *   IDLE -> 收到 START 帧(带总大小) -> 擦除暂存区 -> RECV（回 ACK_READY）
 *   RECV -> 收 DATA 帧逐帧写入 W25Q32（每帧回 ACK）
 *   RECV -> 收 END 帧(带整包CRC32) -> 校验 -> 写元数据+启动标志 -> 回 ACK_OK -> 复位
 *   RECV 中超过 3 秒没收到下一帧 -> 放弃，回 IDLE
 ***************************************************************************************************/
#define OTA_STATE_IDLE  0
#define OTA_STATE_RECV  1

static uint8_t  s_state      = OTA_STATE_IDLE;
static uint32_t s_expect_len = 0;   /* START 帧里告知的固件总大小 */
static uint32_t s_recv_len   = 0;   /* 已接收长度 */
static TickType_t s_last_rx  = 0;   /* 最后一次收到数据的时间 */

/* 回一帧应答：命令字 OTA_CMD_ACK，数据 1 字节状态 */
static void Ota_SendAck(uint8_t status)
{
    uint8_t frame[8];
    uint16_t crc;

    frame[0] = OTA_FRAME_HEAD_0;
    frame[1] = OTA_FRAME_HEAD_1;
    frame[2] = OTA_CMD_ACK;
    frame[3] = 1;      /* 长度低字节 */
    frame[4] = 0;      /* 长度高字节 */
    frame[5] = status;
    crc = CRC16_Modbus(&frame[2], 4);
    frame[6] = (uint8_t)(crc);
    frame[7] = (uint8_t)(crc >> 8);

    Debug_UART_Write(frame, 8);
}

/* 写 EEPROM 启动标志（3 字节：标志 + 密钥） */
static void Ota_WriteFlag(uint8_t flag)
{
    uint8_t data[3];

    data[0] = flag;
    data[1] = EEPROM_KEY_H;
    data[2] = EEPROM_KEY_L;
    W24C02_WriteBytes(EEPROM_FLAG_ADDR, data, 3);
}

/* 新 APP 首次启动确认：读到 PENDING 说明是刚升级上来的，
 * 能跑到这里说明 RTOS 起来了，等 3 秒系统稳定后改写 CONFIRMED，
 * 否则下次复位 Bootloader 会回滚到旧版本 */
static void Ota_BootConfirm(void)
{
    uint8_t data[3];

    W24C02_ReadBytes(EEPROM_FLAG_ADDR, data, 3);
    if (data[1] == EEPROM_KEY_H && data[2] == EEPROM_KEY_L && data[0] == OTA_FLAG_PENDING)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
        Ota_WriteFlag(OTA_FLAG_CONFIRMED);
        Debug_Printf("ota: new app confirmed\n");
    }
}

void App_Ota_Init(void)
{
    W25Q32_Init();
    W24C02_Init();
}

/* 处理 START 帧：data[0..3] = 固件总大小（低字节在前） */
static void Ota_OnStart(const OtaFrame_t *frame)
{
    s_expect_len = (uint32_t)frame->buf[5] | ((uint32_t)frame->buf[6] << 8) |
                   ((uint32_t)frame->buf[7] << 16) | ((uint32_t)frame->buf[8] << 24);

    if (s_expect_len < 512 || s_expect_len > OTA_APP_MAX_SIZE)
    {
        Ota_SendAck(OTA_ACK_FAIL);
        return;
    }

    /* 通知控制任务停车 */
    xEventGroupSetBits(g_event_system, EVT_OTA_MODE);

    /* 擦除暂存区（最多 64 个扇区，约 2~3 秒，期间上位机要等待） */
    Debug_Printf("ota: erase stage, size=%lu\n", s_expect_len);
    W25Q32_EraseRange(W25Q32_STAGE_ADDR, s_expect_len);

    s_recv_len = 0;
    s_last_rx  = xTaskGetTickCount();
    s_state    = OTA_STATE_RECV;

    Ota_SendAck(OTA_ACK_READY);
    Debug_Printf("ota: ready\n");
}

/* 处理 DATA 帧：顺序写入暂存区 */
static void Ota_OnData(const OtaFrame_t *frame)
{
    uint16_t data_len = (uint16_t)(frame->buf[3] | (frame->buf[4] << 8));

    if (s_state != OTA_STATE_RECV)
    {
        return;
    }
    if (s_recv_len + data_len > s_expect_len)
    {
        return;   /* 超出声明大小，丢弃 */
    }

    W25Q32_Write(W25Q32_STAGE_ADDR + s_recv_len, &frame->buf[5], data_len);
    s_recv_len += data_len;
    s_last_rx   = xTaskGetTickCount();

    Ota_SendAck(OTA_ACK_READY);   /* 回 ACK，上位机再发下一帧 */
}

/* 处理 END 帧：data[0..3] = 整包 CRC32（低字节在前） */
static void Ota_OnEnd(const OtaFrame_t *frame)
{
    uint32_t crc;
    uint32_t calc;
    uint32_t i;
    uint8_t  buf[256];
    uint8_t  meta[8];

    if (s_state != OTA_STATE_RECV)
    {
        return;
    }

    crc = (uint32_t)frame->buf[5] | ((uint32_t)frame->buf[6] << 8) |
          ((uint32_t)frame->buf[7] << 16) | ((uint32_t)frame->buf[8] << 24);

    /* 长度必须先对上 */
    if (s_recv_len != s_expect_len)
    {
        Debug_Printf("ota: len mismatch %lu/%lu\n", s_recv_len, s_expect_len);
        goto fail;
    }

    /* 分段回读暂存区算 CRC32 */
    calc = 0xFFFFFFFF;
    for (i = 0; i < s_recv_len; i += sizeof(buf))
    {
        uint32_t once = s_recv_len - i;
        if (once > sizeof(buf))
        {
            once = sizeof(buf);
        }
        W25Q32_Read(W25Q32_STAGE_ADDR + i, buf, once);
        calc = CRC32_Append(calc, buf, once);
    }
    calc = ~calc;

    if (calc != crc)
    {
        Debug_Printf("ota: crc fail calc=0x%08lX expect=0x%08lX\n", calc, crc);
        goto fail;
    }

    /* 写元数据：大小 + CRC，低字节在前 */
    meta[0] = (uint8_t)(s_recv_len);
    meta[1] = (uint8_t)(s_recv_len >> 8);
    meta[2] = (uint8_t)(s_recv_len >> 16);
    meta[3] = (uint8_t)(s_recv_len >> 24);
    meta[4] = (uint8_t)(crc);
    meta[5] = (uint8_t)(crc >> 8);
    meta[6] = (uint8_t)(crc >> 16);
    meta[7] = (uint8_t)(crc >> 24);
    W25Q32_EraseSector(W25Q32_META_ADDR);
    W25Q32_Write(W25Q32_META_ADDR, meta, 8);

    /* 写启动标志 = UPDATE，回复上位机，复位进入 Bootloader 更新 */
    Ota_WriteFlag(OTA_FLAG_UPDATE);

    Ota_SendAck(OTA_ACK_OK);
    Debug_Printf("ota: crc ok, reset to update\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    NVIC_SystemReset();
    return;

fail:
    Ota_SendAck(OTA_ACK_FAIL);
    s_state = OTA_STATE_IDLE;
    xEventGroupClearBits(g_event_system, EVT_OTA_MODE);
}

void App_Ota_Task(void *param)
{
    OtaFrame_t frame;
    uint8_t    cmd;

    (void)param;

    /* 启动确认要在任务里做（需要调度器已运行） */
    Ota_BootConfirm();

    for (;;)
    {
        /* 有帧处理帧，没帧每 100ms 检查一次接收超时 */
        if (xQueueReceive(g_queue_ota_rx, &frame, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            cmd = frame.buf[2];

            if (cmd == OTA_CMD_START)
            {
                Ota_OnStart(&frame);
            }
            else if (cmd == OTA_CMD_DATA)
            {
                Ota_OnData(&frame);
            }
            else if (cmd == OTA_CMD_END)
            {
                Ota_OnEnd(&frame);
            }
        }

        /* 接收中途超时：放弃本次升级 */
        if (s_state == OTA_STATE_RECV &&
            (xTaskGetTickCount() - s_last_rx) > pdMS_TO_TICKS(OTA_RX_TIMEOUT_MS))
        {
            Debug_Printf("ota: rx timeout, abort\n");
            s_state = OTA_STATE_IDLE;
            xEventGroupClearBits(g_event_system, EVT_OTA_MODE);
        }
    }
}
