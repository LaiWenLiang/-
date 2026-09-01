#include "bsp_oidmotor.h"

OID_Feedback_t g_oid_fb[3] = {0};

/* 心跳：DATA[0]=0x00。发送周期应为上位机配置超时时间的 1/2 */
void OID_Heartbeat(uint8_t id)
{
    uint8_t d[1] = {OID_CMD_HEARTBEAT};
    BSP_CAN_Send(id, d, 1);
}

/* 电流控制（恒扭矩）：ma10 单位 10mA，正转正电流 */
void OID_SetCurrent(uint8_t id, int16_t ma10)
{
    uint8_t d[3];
    d[0] = OID_CMD_SET_CURRENT;
    d[1] = (uint8_t)(ma10 >> 8);
    d[2] = (uint8_t)(ma10 & 0xFF);
    BSP_CAN_Send(id, d, 3);
}

/* 刹车（再生制动）：仅正值有效。比放空安全，急停使用 */
void OID_Brake(uint8_t id, uint16_t ma10)
{
    uint8_t d[3];
    d[0] = OID_CMD_BRAKE;
    d[1] = (uint8_t)(ma10 >> 8);
    d[2] = (uint8_t)(ma10 & 0xFF);
    BSP_CAN_Send(id, d, 3);
}

void OID_QuerySpeed(uint8_t id)
{
    uint8_t d[2] = {OID_CMD_QUERY, OID_QUERY_SPEED};
    BSP_CAN_Send(id, d, 2);
}

void OID_QueryFault(uint8_t id)
{
    uint8_t d[2] = {OID_CMD_QUERY, OID_QUERY_FAULT};
    BSP_CAN_Send(id, d, 2);
}

/* 解析驱动器应答帧（DATA[0]=0x0F, DATA[1]=查询项, 数据大端） */
void OID_ParseFrame(const CanFrame_t *frame)
{
    OID_Feedback_t *fb;

    if (frame->id > 2 || frame->len < 2 || frame->data[0] != OID_CMD_QUERY)
    {
        return;
    }
    fb = &g_oid_fb[frame->id];

    switch (frame->data[1])
    {
    case OID_QUERY_SPEED:
        if (frame->len >= 6)
        {
            fb->speed_erpm  = ((int32_t)frame->data[2] << 24) | ((int32_t)frame->data[3] << 16) |
                              ((int32_t)frame->data[4] << 8)  | frame->data[5];
            fb->last_update = xTaskGetTickCount();
        }
        break;

    case OID_QUERY_FAULT:
        if (frame->len >= 4)
        {
            fb->fault = frame->data[3];
        }
        break;

    case OID_QUERY_CURRENT:
        if (frame->len >= 4)
        {
            fb->current_ma10 = (int16_t)((frame->data[2] << 8) | frame->data[3]);
        }
        break;

    case OID_QUERY_VOLTAGE:
        if (frame->len >= 4)
        {
            fb->voltage_v = (uint16_t)((frame->data[2] << 8) | frame->data[3]);
        }
        break;

    default:
        break;
    }
}
