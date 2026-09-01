#ifndef __PID_H
#define __PID_H

/***************************************************************************************************
 * 通用增强型位置式 PID
 * 在基础位置式 PID 之上增加：
 *   - 前馈控制（Kff * Target，减小跟随滞后）
 *   - 积分分离（|误差| > IntSepThreshold 时切除积分，防大误差积分暴涨）
 *   - 积分限幅（IntMax，抗积分饱和）
 *   - 输出死区（|输出| < DeadZone 时归零，防低速爬行/抖动）
 *   - 输出限幅（OutMax/OutMin）
 * 结构体字段全开放，便于上位机/串口在线调参。
 ***************************************************************************************************/
typedef struct
{
    float Target;          /* 目标值 */
    float Actual;          /* 实际值（反馈） */
    float Out;             /* 输出 */

    float Kp;
    float Ki;
    float Kd;
    float Kff;             /* 前馈系数 */

    float Error0;          /* 本次误差 */
    float Error1;          /* 上次误差 */
    float ErrorInt;        /* 误差积分 */

    float IntSepThreshold; /* 积分分离阈值：|err|超过它则不积分；0 = 不启用 */
    float IntMax;          /* 积分限幅：0 = 不限 */
    float DeadZone;        /* 输出死区：|out|小于它则输出0；0 = 不启用 */

    float OutMax;
    float OutMin;
} PID_t;

void  PID_Init(PID_t *p);
void  PID_Update(PID_t *p);
void  PID_Reset(PID_t *p);   /* 清积分与历史误差（模式切换/停车时调用） */

#endif /* __PID_H */
