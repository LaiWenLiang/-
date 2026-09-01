#ifndef __APP_DEBUG_H
#define __APP_DEBUG_H

#include "app.h"

/***************************************************************************************************
 * USART1 调试模块（VOFA+ 上位机）
 *  - 命令通道（JustEngine 协议）：slider,名字,值 / button,动作,状态
 *  - 波形通道（FireWater 协议）：printf("v1,v2,...\n")，VOFA 里勾选 FireWater 即可看曲线
 ***************************************************************************************************/

void APP_Debug_Init(uint32_t baudrate);
void APP_Debug_Process(void);   /* 处理收到的命令（由调试任务周期调用） */
void APP_Debug_Telemetry(void); /* 发送波形数据（由调试任务周期调用） */

/* 波形输出开关：button,wave,1 开 / button,wave,0 关 */
extern uint8_t g_debug_wave_en;

#endif /* __APP_DEBUG_H */
