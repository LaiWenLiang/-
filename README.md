# 工业级智能跟随车（STM32F407 + FreeRTOS）

基于 **STM32F407VET6** 的智能跟随车控制系统：UWB-AOA 跟随 + TOF 激光避障 + 无刷电机串级 PID + 双通道 OTA 远程升级（三段式 Bootloader，支持断电回滚）。

> 本工程由 STM32F103 标准库版本重构升级而来，从裸机任务调度迁移到 FreeRTOS，主控从 F103 升级到 F407。

---

## 功能一览

| 功能 | 说明 |
|---|---|
| UWB 跟随 | UWB-X5-AOA-N 基站解算目标角度/距离，距离环+角度环串级控制 |
| 激光避障 | 4 路 TOF250（前左/前中左/前中右/前右），分级避障 + 紧急制动 |
| 电机控制 | 欧艾迪无刷电机 × 2，CAN 总线（500K），速度环 PID（前馈/积分分离/积分限幅/输出死区） |
| 数据滤波 | UWB 角度/距离卡尔曼滤波，参数可在线调 |
| 在线调参 | 蓝牙 + VOFA+：PID 参数/卡尔曼 Q/R 滑条调节，波形实时观测 |
| 电量检测 | 48V 电池组分压采样（ADC），电压查表估算电量，三级低电保护（语音告警→强制停车） |
| 人机交互 | OLED 状态显示、3 个物理按键、语音播报模块 |
| OTA 升级 | 双通道：RS485 有线 / ESP8266 + OneNET WiFi 远程，三段式 Bootloader（暂存→校验→回滚） |

## 技术栈

`FreeRTOS（8 任务 + 队列/信号量/事件组/任务通知/软件定时器）` `串级 PID` `卡尔曼滤波` `CAN 2.0` `UWB AOA` `I2C（硬件+软件模拟）` `SPI Flash 掉电存储` `EEPROM 启动标志` `IAP/OTA（双区备份回滚）` `MQTT/HTTP（OneNET）` `HMAC-SHA1 鉴权签名` `DMA 双工串口` `CRC16/CRC32` `VOFA+ 在线调参`

## 目录结构

```
├── app/                 # APP 工程（Keil：app/USER/app.uvprojx，链接地址 0x08020000）
│   ├── APP/             # 应用层：任务、串级PID、避障决策、UWB解析、OTA、VOFA调参
│   ├── BSP/             # 板级驱动：TOF/OLED/电机/按键/电池/W25Q32/W24C02/ESP8266
│   ├── SYSTEM/          # 系统层：三个串口统一入口（USART1 蓝牙/RS485 复用）
│   ├── FreeRTOS/        # FreeRTOS 内核
│   └── FWLIB/           # STM32F4 标准外设库
├── bootloader/          # Bootloader 工程（0x08000000，64KB，三段式升级+回滚）
├── tools/pack_ota.py    # RS485 升级固件打包脚本（bin → 带帧头/CRC 的 .ota）
└── 上位机OTA协议.md      # OTA 通信协议文档
```

## 硬件资源分配

| 外设 | 引脚 | 用途 |
|---|---|---|
| USART1 | PA9/PA10（DE=PD3） | 蓝牙(VOFA)/RS485(OTA) 二选一复用，KEY3 切换 |
| USART2 | PA2/PA3 | UWB 基站（DMA 循环 + 空闲中断） |
| USART3 | PC10/PC11 | 语音播报模块 |
| UART5 | PC12/PD2（RST=PD0） | ESP8266 WiFi |
| CAN1 | PB8/PB9 | 无刷电机驱动器 × 2（ID=1 左 / 2 右） |
| I2C2 | PB10/PB11 | TOF250 × 4 |
| 软件I2C | PE0/PE1 | OLED |
| 软件I2C | PF6/PF7 | W24C02（OTA 启动标志） |
| SPI2 | PB12~PB15 | W25Q32（固件暂存 + 旧版备份） |
| ADC1_IN5 | PA5 | 电池电压（180K+10K 分压，19 倍） |
| GPIO | PA0/PA1/PA4 | KEY1/KEY2/KEY3（低电平触发） |
| GPIO | PF9/PF10/PF8 | LED1/LED2/蜂鸣器 |

## OTA 升级（三段式，断电安全）

```
内部 Flash:  [Bootloader 64KB] [预留 64KB] [APP 256KB] [预留 128KB]
W25Q32:      [元数据 8B] [新固件暂存 256KB] [旧 APP 备份 256KB]
W24C02:      启动标志 UPDATE / PENDING / CONFIRMED → 新 APP 起不来自动回滚旧版本
```

- **有线**：`tools/pack_ota.py` 打包 bin → 串口助手分包发送（协议见《上位机OTA协议.md》）
- **无线**：OneNET 后台上传固件 → 车端轮询发现 → OLED 提示 → KEY1 确认 → 下载校验 → 复位升级

## 编译

1. Keil MDK5 打开 `app/USER/app.uvprojx`，编译后自动生成 `Objects/app.bin`（已按 0x08020000 链接）
2. 打开 `bootloader/USER/bootloader.uvprojx` 编译 Bootloader
3. 先烧 Bootloader，再烧 APP（或只烧 Bootloader，用 OTA 装 APP）

> ⚠️ `app/APP/app_onenet_ota.h` 中的 OneNET 设备密钥在开源前已抹除（`"......"`），自行编译需填回真实密钥；WiFi 名称密码同为占位符。
