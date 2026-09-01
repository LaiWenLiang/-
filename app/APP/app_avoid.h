#ifndef __APP_AVOID_H
#define __APP_AVOID_H

#include "app.h"
#include "bsp_tof.h"

/* 避障阈值（cm） */
#define AVOID_ESTOP_CM     20    /* 急停：最后一道保险 */
#define AVOID_WARN_CM      50    /* 警戒区 */
#define AVOID_TURN_OUT_CM  80    /* 中距离转向避让 */
#define AVOID_TURN_MID_CM  70
#define AVOID_BACK_OUT_CM  25    /* 近距离倒车 */
#define AVOID_BACK_MID_CM  22

/* 多级避障状态机（移植自 F103 工程 TOF10120_Judgment） */
TurnDirection_t Avoid_Judge(const TOF_Distance_t *tof);

#endif /* __APP_AVOID_H */
