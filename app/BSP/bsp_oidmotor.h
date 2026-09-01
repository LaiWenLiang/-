#ifndef __BSP_OIDMOTOR_H
#define __BSP_OIDMOTOR_H

#include "bsp.h"
#include "bsp_can.h"

/***************************************************************************************************
 * 欧艾迪 FOC 无刷驱动器 CAN 协议封装（编程手册 V1.2）
 * 标准帧，CAN ID = 驱动器 ID，DATA[0] = 指令码
 ***************************************************************************************************/
#define OID_ID_LEFT    1    /* 左轮驱动器 ID */
#define OID_ID_RIGHT   2    /* 右轮驱动器 ID */

/* 指令码 */
#define OID_CMD_HEARTBEAT     0x00   /* 心跳（必须周期发送，否则驱动器放空电机！） */
#define OID_CMD_SET_CURRENT   0x01   /* 电流控制 int16, 单位10mA */
#define OID_CMD_SET_SPEED     0x02   /* 转速控制 int32, 单位erpm（驱动器内部速度环，本项目不用） */
#define OID_CMD_SET_DUTY      0x03   /* 占空比 int16, -1000~1000 */
#define OID_CMD_BRAKE         0x08   /* 刹车电流 int16, 单位10mA（再生制动） */
#define OID_CMD_QUERY         0x0F   /* 查询 */

/* 查询项 */
#define OID_QUERY_FAULT       0x00
#define OID_QUERY_SPEED       0x01   /* 返回 erpm */
#define OID_QUERY_DUTY        0x02
#define OID_QUERY_VOLTAGE     0x04
#define OID_QUERY_CURRENT     0x05
#define OID_QUERY_TEMP        0x07

/* 反馈数据（每个驱动器一份，由 Task_MotorCan 更新） */
typedef struct
{
    int32_t  speed_erpm;    /* 当前转速 erpm（rpm = erpm / 磁极对数） */
    int16_t  current_ma10;  /* 电机电流 10mA */
    uint16_t voltage_v;     /* 母线电压 V */
    uint8_t  fault;         /* 故障码，0=无故障 */
    TickType_t last_update; /* 反馈时间戳（失联检测用） */
} OID_Feedback_t;

extern OID_Feedback_t g_oid_fb[3];   /* 下标即驱动器 ID，0 不用 */

void    OID_Heartbeat(uint8_t id);
void    OID_SetCurrent(uint8_t id, int16_t ma10);
void    OID_Brake(uint8_t id, uint16_t ma10);
void    OID_QuerySpeed(uint8_t id);
void    OID_QueryFault(uint8_t id);
void    OID_ParseFrame(const CanFrame_t *frame);

#endif /* __BSP_OIDMOTOR_H */
