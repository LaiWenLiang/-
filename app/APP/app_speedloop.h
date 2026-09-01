#ifndef __APP_SPEEDLOOP_H
#define __APP_SPEEDLOOP_H

#include "app.h"
#include "pid.h"
#include <stdint.h>

/* 磁极对数（按电机实际参数修改；rpm = erpm / 磁极对数） */
#define MOTOR_POLE_PAIRS      4

/* 速度环参数（台架调试时调整） */
#define SPEEDLOOP_KP          0.8f    /* 比例：误差1erpm -> 0.8mA 量级 */
#define SPEEDLOOP_KI          0.15f   /* 积分 */
#define SPEEDLOOP_MAX_MA10    2000    /* 输出限幅 2000*10mA = 20A，按驱动器额定修改 */
#define SPEEDLOOP_INTE_MAX    8000    /* 积分限幅（抗饱和） */

/* 目标转速上限（erpm），速度指令映射用 */
#define SPEEDLOOP_MAX_ERPM    8000

/* 内环：速度环（左右轮各一个增强型 PID 实例） */
PID_t *SpeedLoop_GetPID(uint8_t id);

void   SpeedLoop_Reset(void);
int16_t SpeedLoop_Step(uint8_t id, int32_t target_erpm, int32_t feedback_erpm);

#endif /* __APP_SPEEDLOOP_H */
