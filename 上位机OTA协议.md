# 跟随车 OTA 升级协议（上位机对接文档）

> ⚠️ CAN-OTA 通道（0x7A0~0x7B0）已废弃，CAN 总线现在只跑电机驱动器。
> 当前有线升级走 **RS485**（与蓝牙调试口共用 USART1，KEY3 切换模式）；
> 无线升级走 OneNET WiFi（ESP8266），检测到新固件后需按 KEY1 确认。

## 1. 物理层

- RS485 接 USART1（PA9 TX / PA10 RX），DE 方向脚由车端自动控制
- 波特率 **115200**，8N1
- 升级前车端要把 USART1 切到 **RS485 模式**（OLED 显示 `MODE: RS485(OTA)`，KEY3 切换）
- 升级期间车端自动停车，升级完成后自动复位

## 2. 帧格式（小端，低字节在前）

```
[0]    0x55        帧头
[1]    0xAA        帧头
[2]    CMD         命令字
[3..4] LEN         数据长度 N（0~255）
[5..]  DATA[N]     数据
[5+N]  CRC16_L     CRC16 低字节
[6+N]  CRC16_H     CRC16 高字节
```

CRC16：Modbus，多项式 `0xA001`，初值 `0xFFFF`，计算范围 = **CMD + LEN(2字节) + DATA**

命令字：

| CMD | 方向 | 含义 |
|---|---|---|
| `0x01` START | 上位机→车 | 开始升级，DATA = 固件总大小（4 字节，小端） |
| `0x02` DATA  | 上位机→车 | 固件数据（≤255 字节/帧，按顺序发） |
| `0x03` END   | 上位机→车 | 结束，DATA = 整包 CRC32（4 字节，小端） |
| `0x80` ACK   | 车→上位机 | 应答，DATA = 1 字节状态 |

ACK 状态值：

| 值 | 含义 |
|---|---|
| `0xAA` | 就绪 / 本帧数据已写入，可以发下一帧 |
| `0x55` | 校验通过，即将复位升级 |
| `0xEE` | 校验失败 / 参数非法，本次作废 |

## 3. 升级流程

```
上位机                          车端 APP
  |------ START(总大小) ------->|  擦除 W25Q32 暂存区（约 2~3 秒）
  |<----- ACK(0xAA) ------------|  准备就绪
  |------ DATA 帧 ------------->|  写入暂存区
  |<----- ACK(0xAA) ------------|  每帧都要等 ACK 再发下一帧
  |        ……重复…………           |
  |------ END(整包CRC32) ------>|  回读校验
  |<----- ACK(0x55) ------------|  500ms 后自动复位进入升级
  （升级约 10~20 秒，之后新 APP 自动运行）
```

失败情况：

- 校验失败：回 `ACK(0xEE)`，本次升级作废，旧 APP 照常运行，可重新发起
- 帧中断超过 3 秒：车端自动放弃，旧 APP 照常运行
- 升级过程断电 / 新 APP 起不来：Bootloader 下次上电自动回滚旧版本

## 4. CRC 约定

- **帧校验 CRC16-Modbus**：多项式 `0xA001`，初值 `0xFFFF`
- **整包 CRC32**：多项式 `0xEDB88320`，初值 `0xFFFFFFFF`，结果取反，与 Python `zlib.crc32()` 一致

Python 示例：

```python
import zlib, serial

def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def pack(cmd, payload=b""):
    body = bytes([cmd, len(payload) & 0xFF, len(payload) >> 8]) + payload
    crc  = crc16_modbus(body)
    return b"\x55\xAA" + body + bytes([crc & 0xFF, crc >> 8])

def send_upgrade(port, bin_path):
    data = open(bin_path, 'rb').read()
    size, crc = len(data), zlib.crc32(data) & 0xFFFFFFFF

    port.write(pack(0x01, size.to_bytes(4, 'little')))
    wait_ack(port, 0xAA, timeout=10)          # 擦除约 2~3 秒

    for i in range(0, size, 255):             # 每帧最多 255 字节
        port.write(pack(0x02, data[i:i+255]))
        wait_ack(port, 0xAA, timeout=3)       # 逐帧等应答

    port.write(pack(0x03, crc.to_bytes(4, 'little')))
    return wait_ack(port, (0x55, 0xEE), timeout=10)
```

## 5. 固件文件说明

- 使用 Keil 编译 APP 工程后自动生成的 **`app\USER\Objects\app.bin`**
- 该 bin 已按 `0x08020000` 链接，直接整文件发送即可，**不要加任何文件头**
- 大小限制：512 字节 ~ 256 KB

## 6. 存储布局（备忘）

| 位置 | 内容 |
|---|---|
| 内部 Flash `0x08000000` 64KB | Bootloader |
| 内部 Flash `0x08020000` 256KB | APP 运行区 |
| W25Q32 `0x000000` 8B | 元数据：固件大小(4B) + CRC32(4B)，小端 |
| W25Q32 `0x001000` 256KB | 新固件暂存区 |
| W25Q32 `0x100000` 256KB | 旧 APP 备份区（回滚用） |
| W24C02 `0x10` 3B | 启动标志 + 密钥 0x5A6B |

启动标志取值：`0x01` 待更新 / `0x02` 无更新 / `0x03` 新APP待确认 / `0x04` 已确认

## 7. 按键与模式切换

| 按键 | 引脚 | 功能 |
|---|---|---|
| KEY1 | PA0 | 升级提示出现时 = 接受下载 |
| KEY2 | PA1 | 升级提示出现时 = 拒绝下载 |
| KEY3 | PA4 | 平时 = 切换 USART1 蓝牙(VOFA)/RS485(OTA) 模式 |

- 低电平触发（按下接地，内部上拉）
- OneNET 检测到新固件时 OLED 显示版本号，60 秒无操作视为拒绝
- 升级中 OLED 显示 `OTA UPGRADING... / DO NOT POWER OFF`
