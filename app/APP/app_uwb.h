#ifndef __APP_UWB_H
#define __APP_UWB_H

#include "app.h"

/* UWB 协议帧：帧头 0x2A，帧尾 0x23（沿用原工程协议） */
#define UWB_CMD_SOP    0x2A
#define UWB_CMD_FOOT   0x23

bool APP_UWB_ParseFrame(const uint8_t *buf, uint16_t len, AOA_Data_t *out);

/* 一维卡尔曼滤波（UWB 角度/距离平滑用） */
void  Kalman_Init(KalmanState_t *k, float q, float r);
float Kalman_Update(KalmanState_t *k, float measurement);

/***************************************************************************************************
 * 跳变滤波（移植自原工程 AOA_Angle_Filter）：拦在卡尔曼前面挡野值
 *  UWB 多径会导致角度/距离瞬间跳变（如 10° 突然变 150°），卡尔曼对这类野值抑制有限；
 *  跳变滤波先判断"这一帧和上一帧差多少"，差太多就保持上次有效值不放行，
 *  连续超限 hold_max 次才认为"是真的变了"予以放行。
 ***************************************************************************************************/
typedef struct
{
    int     last;        /* 上一次放行的有效值 */
    uint8_t hold_cnt;    /* 连续被拦截次数 */
    uint8_t started;     /* 是否已有过有效值 */
} JumpFilter_t;

/* 跳变阈值（VOFA 可调）：同向/异向变化阈值，连续超限放行次数 */
extern int g_jump_same;    /* 同向阈值：角度用°，距离用cm */
extern int g_jump_inv;     /* 异向阈值（数据跨过 0 算异向，角度更有意义） */
extern int g_jump_hold;    /* 连续超限多少次后放行（防止目标真跑了跟不上） */

int UWB_JumpFilter(JumpFilter_t *f, int curr, int same_thresh, int inv_thresh, int hold_max);

#endif /* __APP_UWB_H */
