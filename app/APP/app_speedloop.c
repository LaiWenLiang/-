#include "app_speedloop.h"
#include "pid.h"

/**************************************************************************
 * 内环：速度环（左右轮各一个增强型 PID 实例）
 *   输入：目标 erpm（外环/遥控给定） vs CAN 反馈 erpm
 *   输出：电流指令（10mA），下发驱动器 0x01 电流控制
 **************************************************************************/
PID_t *SpeedLoop_GetPID(uint8_t id);

static PID_t s_pid[3];   /* 下标 = 驱动器 ID（1=左，2=右） */
static uint8_t s_inited = 0;

static void SpeedLoop_InitOnce(void)
{
    uint8_t id;

    if (s_inited) { return; }
    s_inited = 1;

    for (id = 1; id <= 2; id++)
    {
        PID_Init(&s_pid[id]);
        s_pid[id].Kp = SPEEDLOOP_KP;
        s_pid[id].Ki = SPEEDLOOP_KI;
        s_pid[id].Kd = 0;
        s_pid[id].Kff = 0;                  /* 需要前馈时可设 Kff = 前馈增益 */
        s_pid[id].IntSepThreshold = 2000;   /* 误差 >2000erpm 时不积分 */
        s_pid[id].IntMax          = SPEEDLOOP_INTE_MAX;
        s_pid[id].DeadZone        = 0;      /* 电流输出无死区 */
        s_pid[id].OutMax          = SPEEDLOOP_MAX_MA10;
        s_pid[id].OutMin          = -SPEEDLOOP_MAX_MA10;
    }
}

void SpeedLoop_Reset(void)
{
    SpeedLoop_InitOnce();
    PID_Reset(&s_pid[1]);
    PID_Reset(&s_pid[2]);
}

int16_t SpeedLoop_Step(uint8_t id, int32_t target_erpm, int32_t feedback_erpm)
{
    PID_t *p;

    SpeedLoop_InitOnce();
    if (id > 2)
    {
        return 0;
    }

    p = &s_pid[id];

    /* 目标为零：清积分并直接输出 0，防止松手后积分残余导致窜动 */
    if (target_erpm == 0)
    {
        PID_Reset(p);
        return 0;
    }

    p->Target = (float)target_erpm;
    p->Actual = (float)feedback_erpm;
    PID_Update(p);

    return (int16_t)p->Out;
}

/* 供调试层在线调参：取左/右轮速度环 PID 实例 */
PID_t *SpeedLoop_GetPID(uint8_t id)
{
    SpeedLoop_InitOnce();
    return (id >= 1 && id <= 2) ? &s_pid[id] : NULL;
}
