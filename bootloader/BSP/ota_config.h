#ifndef __OTA_CONFIG_H
#define __OTA_CONFIG_H

/***************************************************************************************************
 * OTA 三段式升级 公共配置（BOOTLOADER 工程与 APP 工程共用，两边必须保持一致！）
 *
 * 内部 Flash 分区（STM32F407VET6，512KB）：
 *   Sector 0~3  0x08000000  64KB   Bootloader
 *   Sector 4    0x08010000  64KB   预留
 *   Sector 5~6  0x08020000  256KB  APP 运行区
 *   Sector 7    0x08060000  128KB  预留
 *
 * W25Q32（外部 SPI Flash，4MB）：
 *   0x000000  8字节  元数据：新固件大小(4字节,低字节在前) + CRC32(4字节,低字节在前)
 *   0x001000  256KB  新固件暂存区（上位机 CAN 发来的固件先存这里）
 *   0x100000  256KB  旧 APP 备份区（升级前备份，回滚时用）
 *
 * W24C02（EEPROM）地址 0x10 起 3 字节：
 *   [0] 启动标志（见下面 OTA_FLAG_xxx）
 *   [1] 密钥高字节 0x5A
 *   [2] 密钥低字节 0x6B
 ***************************************************************************************************/

/* ---------- 内部 Flash 地址 ---------- */
#define OTA_BOOT_ADDR        0x08000000UL   /* Bootloader 起始 */
#define OTA_APP_ADDR         0x08020000UL   /* APP 运行区起始（Sector 5） */
#define OTA_APP_MAX_SIZE     (256UL * 1024) /* APP 最大 256KB */

/* APP 运行区占用 Sector 5 和 Sector 6 */
#define OTA_APP_SECTOR_1     FLASH_Sector_5
#define OTA_APP_SECTOR_2     FLASH_Sector_6

/* ---------- W25Q32 地址 ---------- */
#define W25Q32_META_ADDR     0x000000UL     /* 元数据（8字节） */
#define W25Q32_STAGE_ADDR    0x001000UL     /* 新固件暂存区 */
#define W25Q32_BACKUP_ADDR   0x100000UL     /* 旧 APP 备份区 */

/* ---------- W24C02 启动标志 ---------- */
#define EEPROM_FLAG_ADDR     0x10           /* 标志存放地址 */
#define EEPROM_KEY_H         0x5A           /* 密钥高字节 */
#define EEPROM_KEY_L         0x6B           /* 密钥低字节 */

/* 启动标志取值 */
#define OTA_FLAG_UPDATE      0x01           /* 有新固件，需要更新 */
#define OTA_FLAG_NO_UPDATE   0x02           /* 无更新，直接启动 APP */
#define OTA_FLAG_PENDING     0x03           /* 新 APP 待确认（下次启动还没确认就回滚） */
#define OTA_FLAG_CONFIRMED   0x04           /* 新 APP 已确认能正常运行 */

/* ---------- CAN 升级协议帧 ID（标准帧） ---------- */
#define OTA_CAN_ID_CMD       0x7A0          /* 上位机 -> 车：升级命令，[0]=0x01, [1..4]=固件总大小(低字节在前) */
#define OTA_CAN_ID_DATA      0x7A1          /* 上位机 -> 车：固件数据帧，每帧最多 8 字节，按顺序连发 */
#define OTA_CAN_ID_END       0x7A2          /* 上位机 -> 车：结束帧，[0..3]=整包CRC32(低字节在前) */
#define OTA_CAN_ID_ACK       0x7B0          /* 车 -> 上位机：应答，[0]=状态（见下） */

/* 应答状态 */
#define OTA_ACK_READY        0xAA           /* 暂存区擦除完成，可以发数据 */
#define OTA_ACK_OK           0x55           /* 校验通过，即将重启升级 */
#define OTA_ACK_FAIL         0xEE           /* 校验失败，本次升级作废 */

/* 接收超时：超过该时间没收到下一帧就放弃本次升级 */
#define OTA_RX_TIMEOUT_MS    3000

#endif /* __OTA_CONFIG_H */
