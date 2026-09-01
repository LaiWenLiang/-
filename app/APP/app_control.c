#include "app_control.h"
#include "app_speedloop.h"
#include "bsp_oidmotor.h"
#include "pid.h"

/**************************************************************************
 * 控制执行层（无刷电机 CAN 版）
 * 指令 -> 左右轮目标转速(erpm) -> 速度环PI -> 电流指令 -> CAN 下发
 *
 * 差速底盘：左轮 = 驱动器ID 1，右轮 = 驱动器ID 2
 * 正转速 = 前进
 *
 * 【如需换回有刷电机】：
 *   1. 取消 bsp.c 中 BSP_Motor_Init() 的注释；
 *   2. 把本文件替换为原 PWM 版本（git/备份中取回）；
 *   3. Keil 工程中保留的 bsp_motor.c 即有刷驱动。
 **************************************************************************/

/* 速度档位(PWM量纲 0~5500) 映射到目标 erpm */
static int32_t Speed_To_Erpm(uint16_t speed)
{
    if (speed == 0)
    {
        return 0;
    }
    return (int32_t)speed * SPEEDLOOP_MAX_ERPM / MOTOR_PWM_FULLSCALE;
}

/* 双轮闭环输出：目标 erpm -> 速度环 -> 电流下发 */
static void Motor_CloseLoop(int32_t target_l, int32_t target_r)
{
    int16_t cur_l = SpeedLoop_Step(OID_ID_LEFT,  target_l, g_oid_fb[OID_ID_LEFT].speed_erpm);
    int16_t cur_r = SpeedLoop_Step(OID_ID_RIGHT, target_r, g_oid_fb[OID_ID_RIGHT].speed_erpm);

    g_target_erpm_l = target_l;   /* 供 VOFA 波形显示 */
    g_target_erpm_r = target_r;

    OID_SetCurrent(OID_ID_LEFT,  cur_l);
    OID_SetCurrent(OID_ID_RIGHT, cur_r);
}

/* 主动刹停（急停用）：再生制动，比放空安全 */
void Control_EmergencyBrake(void)
{
    SpeedLoop_Reset();
    OID_Brake(OID_ID_LEFT,  OID_BRAKE_MA10);
    OID_Brake(OID_ID_RIGHT, OID_BRAKE_MA10);
}

void Control_Execute(TurnDirection_t cmd, uint16_t speed)
{
    int32_t v = Speed_To_Erpm(speed);
    int32_t t = Speed_To_Erpm(TURN_SPEED);

    switch (cmd)
    {
    case RCSF:
    case FOLLOW_STRAIGHT:
    case FOLLOW_CAR:
        Motor_CloseLoop(v, v);
        break;

    case BACKWARD:
        Motor_CloseLoop(-v, -v);
        break;

    case SPOT_LEFT_TURN:
        Motor_CloseLoop(t, -t);    /* 左轮前进右轮后退 -> 视实际转向方向可调换符号 */
        break;

    case SPOT_RIGHT_TURN:
        Motor_CloseLoop(-t, t);
        break;

    case LEFT_BACKWARD:            /* 左后倒：右轮倒退 */
        Motor_CloseLoop(0, -Speed_To_Erpm(BACK_SPEED));
        break;

    case RIGHT_BACKWARD:           /* 右后倒：左轮倒退 */
        Motor_CloseLoop(-Speed_To_Erpm(BACK_SPEED), 0);
        break;

    case STOP_MOTOR:
    default:
        Motor_CloseLoop(0, 0);     /* 速度环目标 0 = 主动停稳（优于放空） */
        break;
    }
}

/**************************************************************************
 * 串级跟随控制（外环 + 角度分配）：
 *   外环：距离 PID —— UWB距离误差 -> 目标车速（cm -> 速度指令量纲）
 *   角度：P 环   —— UWB角度偏差 -> 左右轮差速量
 *   合成：左轮 = 车速 - 差速，右轮 = 车速 + 差速 -> 各自进速度环内环
 **************************************************************************/
#define DIST_KP            30.0f    /* 距离误差1cm -> 30 速度指令单位 */
#define DIST_KI            0.5f
#define DIST_KD            5.0f     /* 人突然减速时提前刹车 */
#define DIST_KFF           0.0f     /* 前馈（人的速度），可调 */
#define DIST_INTE_SEP      50.0f    /* 误差>50cm 不积分（防猛冲） */
#define DIST_DEADBAND      5.0f     /* ±5cm 死区，人站定车不抖 */

#define ANGLE_KP           20.0f    /* 角度偏差1° -> 20 差速单位 */
#define ANGLE_DEADBAND     5.0f     /* ±5° 死区，治摇头 */
#define ANGLE_SPOT         60.0f    /* 超过60°先原地转向对准 */

/* ---- 可调参数（默认值为宏，VOFA 在线调参直接改这些全局变量） ---- */
float    g_dist_kp       = DIST_KP;
float    g_angle_kp      = ANGLE_KP;
float    g_angle_deadband = ANGLE_DEADBAND;
float    g_angle_spot    = ANGLE_SPOT;
float    g_dist_deadband = DIST_DEADBAND;
uint16_t g_follow_distance = FOLLOW_DISTANCE_CM;

static PID_t s_pid_dist;
static uint8_t s_dist_inited = 0;

PID_t *Control_GetDistPID(void)
{
    return &s_pid_dist;
}

void Control_FollowStep(const AOA_Data_t *aoa)
{
    float angle, abs_ang, diff;
    float speed_cmd;
    int32_t target_l, target_r;

    if (!s_dist_inited)
    {
        s_dist_inited = 1;
        PID_Init(&s_pid_dist);
        s_pid_dist.Kp = g_dist_kp;
        s_pid_dist.Ki = DIST_KI;
        s_pid_dist.Kd = DIST_KD;
        s_pid_dist.Kff = DIST_KFF;
        s_pid_dist.IntSepThreshold = DIST_INTE_SEP;
        s_pid_dist.IntMax = 200;
        s_pid_dist.DeadZone = 0;             /* 死区放在输出前的误差端手动处理 */
        s_pid_dist.OutMax = FOLLOW_SPEED;
        s_pid_dist.OutMin = -BACK_SPEED * 2; /* 允许小幅后退拉开距离 */
    }

    /* 外环：目标跟随距离 vs 实际距离 */
    s_pid_dist.Target = (float)g_follow_distance;
    s_pid_dist.Actual = (float)aoa->distance;

    /* 距离死区：误差 ±死区 内输出 0 */
    if (s_pid_dist.Actual < s_pid_dist.Target + g_dist_deadband &&
        s_pid_dist.Actual > s_pid_dist.Target - g_dist_deadband)
    {
        PID_Reset(&s_pid_dist);
        speed_cmd = 0;
    }
    else
    {
        PID_Update(&s_pid_dist);
        speed_cmd = s_pid_dist.Out;
    }

    /* 角度通道：P 环 + 死区 */
    angle    = (float)aoa->angle;
    abs_ang  = (angle < 0) ? -angle : angle;

    if (abs_ang < g_angle_deadband)
    {
        diff = 0;
    }
    else if (abs_ang > g_angle_spot)
    {
        /* 大角度：先原地转向对准，不前进 */
        Control_Execute((angle > 0) ? SPOT_LEFT_TURN : SPOT_RIGHT_TURN, 0);
        return;
    }
    else
    {
        diff = g_angle_kp * angle;
    }

    target_l = (int32_t)((speed_cmd - diff) * SPEEDLOOP_MAX_ERPM / MOTOR_PWM_FULLSCALE);
    target_r = (int32_t)((speed_cmd + diff) * SPEEDLOOP_MAX_ERPM / MOTOR_PWM_FULLSCALE);
    Motor_CloseLoop(target_l, target_r);
}

/**************************************************************************
 * 跟随决策（档位式，已弃用，保留参考）：
 * 新代码请使用 Control_FollowStep() 串级控制
 **************************************************************************/
TurnDirection_t Control_FollowDecide(const AOA_Data_t *aoa, uint16_t *speed_out)
{
    int angle    = aoa->angle;
    int distance = aoa->distance;
    int abs_ang  = (angle < 0) ? -angle : angle;

    if (distance <= FOLLOW_DISTANCE_CM + 20)
    {
        return STOP_MOTOR;
    }

    if (distance >= FOLLOW_DISTANCE_CM + 100)
    {
        *speed_out = FOLLOW_SPEED;
    }
    else
    {
        /* 线性减速区 */
        *speed_out = FOLLOW_SPEED * (distance - FOLLOW_DISTANCE_CM - 20) / 80;
        if (*speed_out < 1000) { *speed_out = 1000; }
    }

    if (abs_ang > 25)
    {
        return (angle > 0) ? SPOT_LEFT_TURN : SPOT_RIGHT_TURN;
    }

    return FOLLOW_STRAIGHT;
}
