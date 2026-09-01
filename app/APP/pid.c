#include "pid.h"

void PID_Init(PID_t *p)
{
    p->Target  = 0;
    p->Actual  = 0;
    p->Out     = 0;
    p->Error0  = 0;
    p->Error1  = 0;
    p->ErrorInt = 0;
}

void PID_Reset(PID_t *p)
{
    p->Error0   = 0;
    p->Error1   = 0;
    p->ErrorInt = 0;
    p->Out      = 0;
}

void PID_Update(PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    /* 积分分离：大误差时不积分，防止积分暴涨后过冲 */
    if (p->Ki != 0 &&
        (p->IntSepThreshold == 0 ||
         (p->Error0 < p->IntSepThreshold && p->Error0 > -p->IntSepThreshold)))
    {
        p->ErrorInt += p->Error0;
    }

    /* 积分限幅（抗饱和） */
    if (p->IntMax > 0)
    {
        if (p->ErrorInt > p->IntMax)  { p->ErrorInt = p->IntMax; }
        if (p->ErrorInt < -p->IntMax) { p->ErrorInt = -p->IntMax; }
    }

    p->Out = p->Kp  * p->Error0
           + p->Ki  * p->ErrorInt
           + p->Kd  * (p->Error0 - p->Error1)
           + p->Kff * p->Target;    /* 前馈 */

    /* 输出限幅 */
    if (p->Out > p->OutMax) { p->Out = p->OutMax; }
    if (p->Out < p->OutMin) { p->Out = p->OutMin; }

    /* 输出死区 */
    if (p->DeadZone > 0 && p->Out < p->DeadZone && p->Out > -p->DeadZone)
    {
        p->Out = 0;
    }
}
