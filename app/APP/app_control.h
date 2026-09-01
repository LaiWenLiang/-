#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

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

/* 跟随参数 */
#define FOLLOW_DISTANCE_CM   100   /* 目标跟随距离 */
#define FOLLOW_SPEED         4500
#define TURN_SPEED           2000
#define REMOTE_SPEED         4000
#define BACK_SPEED           700

/* 速度指令满量程（原 PWM 量纲）与急停刹车电流 */
#define MOTOR_PWM_FULLSCALE  5500
#define OID_BRAKE_MA10       1500  /* 急停刹车电流 15A，按驱动器/电源能力调整 */

/* 主动刹停（急停路径调用） */
void Control_EmergencyBrake(void);
/* 运动执行：把运动指令翻译成左右轮目标转速并经速度环输出 */
void Control_Execute(TurnDirection_t cmd, uint16_t speed);
/* 串级跟随：距离外环PID + 角度P差速 + 速度内环（跟随模式调用） */
void Control_FollowStep(const AOA_Data_t *aoa);

/* 供调试层在线调参 */
#include "pid.h"
PID_t   *Control_GetDistPID(void);
extern float    g_dist_kp;
extern float    g_angle_kp;
extern float    g_angle_deadband;
extern float    g_angle_spot;
extern float    g_dist_deadband;
extern uint16_t g_follow_distance;
/* 跟随决策：根据 AOA 角度/距离给出运动指令 */
TurnDirection_t Control_FollowDecide(const AOA_Data_t *aoa, uint16_t *speed_out);

#endif /* __APP_CONTROL_H */
