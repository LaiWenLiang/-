#include "app_debug.h"
#include "app_speedloop.h"
#include "app_control.h"
#include "app_uwb.h"
#include "bsp_oidmotor.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

uint8_t g_debug_wave_en = 0;

void APP_Debug_Init(uint32_t baudrate)
{
    Debug_UART_Init(baudrate);
}

/* ---------------- 波形通道（FireWater）：目标/反馈转速 + 电流 ---------------- */

void APP_Debug_Telemetry(void)
{
    if (!g_debug_wave_en)
    {
        return;
    }
    Debug_Printf("%d,%d,%d,%d,%d,%d\n",
                 (int)g_target_erpm_l, (int)g_oid_fb[OID_ID_LEFT].speed_erpm,
                 (int)g_target_erpm_r, (int)g_oid_fb[OID_ID_RIGHT].speed_erpm,
                 (int)g_oid_fb[OID_ID_LEFT].current_ma10,
                 (int)g_oid_fb[OID_ID_RIGHT].current_ma10);
}

/* ---------------- 命令通道（JustEngine）：滑条调参 / 按钮控制 ---------------- */

static void Apply_Slider(const char *name, float value)
{
    PID_t *sp_l = SpeedLoop_GetPID(OID_ID_LEFT);
    PID_t *sp_r = SpeedLoop_GetPID(OID_ID_RIGHT);
    PID_t *dist = Control_GetDistPID();

    /* 速度环（左右轮同步） */
    if      (strcmp(name, "SpeedKp") == 0)  { sp_l->Kp = sp_r->Kp = value; }
    else if (strcmp(name, "SpeedKi") == 0)  { sp_l->Ki = sp_r->Ki = value; }
    else if (strcmp(name, "SpeedKd") == 0)  { sp_l->Kd = sp_r->Kd = value; }
    else if (strcmp(name, "SpeedFF") == 0)  { sp_l->Kff = sp_r->Kff = value; }
    else if (strcmp(name, "SpeedSep") == 0) { sp_l->IntSepThreshold = sp_r->IntSepThreshold = value; }
    else if (strcmp(name, "SpeedOutMax") == 0)
    {
        sp_l->OutMax = sp_r->OutMax = value;
        sp_l->OutMin = sp_r->OutMin = -value;
    }
    /* 距离外环 */
    else if (strcmp(name, "DistKp") == 0)   { dist->Kp = value; }
    else if (strcmp(name, "DistKi") == 0)   { dist->Ki = value; }
    else if (strcmp(name, "DistKd") == 0)   { dist->Kd = value; }
    else if (strcmp(name, "DistFF") == 0)   { dist->Kff = value; }
    else if (strcmp(name, "DistSep") == 0)  { dist->IntSepThreshold = value; }
    else if (strcmp(name, "DistDZ") == 0)   { g_dist_deadband = value; }
    else if (strcmp(name, "FollowDist") == 0) { g_follow_distance = (uint16_t)value; }
    /* 角度 P 环 */
    else if (strcmp(name, "AngleKp") == 0)  { g_angle_kp = value; }
    else if (strcmp(name, "AngleDZ") == 0)  { g_angle_deadband = value; }
    else if (strcmp(name, "AngleSpot") == 0){ g_angle_spot = value; }
    /* 卡尔曼滤波（UWB 角度/距离） */
    else if (strcmp(name, "KalAngQ") == 0)  { g_kf_uwb_angle.q = value; }
    else if (strcmp(name, "KalAngR") == 0)  { g_kf_uwb_angle.r = value; }
    else if (strcmp(name, "KalDisQ") == 0)  { g_kf_uwb_dist.q = value; }
    else if (strcmp(name, "KalDisR") == 0)  { g_kf_uwb_dist.r = value; }
    /* 跳变滤波（UWB 野值拦截） */
    else if (strcmp(name, "JumpSame") == 0) { g_jump_same = (int)value; }
    else if (strcmp(name, "JumpInv") == 0)  { g_jump_inv  = (int)value; }
    else if (strcmp(name, "JumpHold") == 0) { g_jump_hold = (int)value; }
    else
    {
        printf("unknown slider: %s\n", name);
        return;
    }
    printf("ok %s=%f\n", name, (double)value);
}

static void Apply_Button(const char *action, int state)
{
    if (strcmp(action, "wave") == 0)
    {
        g_debug_wave_en = (uint8_t)state;
        printf("wave %s\n", state ? "on" : "off");
    }
    else if (strcmp(action, "estop") == 0 && state == 1)
    {
        Control_EmergencyBrake();
        printf("estop!\n");
    }
    else if (strcmp(action, "pidreset") == 0 && state == 1)
    {
        SpeedLoop_Reset();
        PID_Reset(Control_GetDistPID());
        printf("pid reset\n");
    }
}

void APP_Debug_Process(void)
{
    static char line[64];
    char *tag;

    if (Debug_UART_ReadLine(line, sizeof(line)) == 0)
    {
        return;
    }

    tag = strtok(line, ",");

    if (tag != NULL && strcmp(tag, "slider") == 0)
    {
        char *name  = strtok(NULL, ",");
        char *value = strtok(NULL, ",");
        if (name != NULL && value != NULL)
        {
            Apply_Slider(name, (float)atof(value));
        }
    }
    else if (tag != NULL && strcmp(tag, "button") == 0)
    {
        char *action = strtok(NULL, ",");
        char *state  = strtok(NULL, ",");
        if (action != NULL && state != NULL)
        {
            Apply_Button(action, atoi(state));
        }
    }
}
