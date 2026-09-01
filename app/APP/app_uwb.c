#include "app_uwb.h"

/**************************************************************************
 * UWB 帧解析（精简移植自原工程 uwb.c）
 * 帧格式: [SOP 0x2A][LEN_H][LEN_L][src(8)][dst(8)][type][direct][payload...][check][0x23]
 * type=0x64/0x61 为 AOA 定位帧，payload 内含 4 组标签 (angle,range)
 *
 * 注：此处提取第 0 组标签的角度/距离作为跟随目标。
 * 完整多标签/遥控位域解析可按原工程结构体继续扩展。
 **************************************************************************/

#pragma pack(push, 1)
typedef struct
{
    uint32_t timer;
    uint16_t anc_addr16;
    uint16_t tag_addr16;
    uint8_t  tag_sn;
    uint8_t  tag_mask;
    struct
    {
        int16_t  angle;
        uint16_t range;
        int16_t  rssi;
    } tag[4];
    uint32_t detail_para;   /* 遥控/电量位域 */
} UWB_AoaPayload_t;
#pragma pack(pop)

/* 遥控位域解析（与原工程 tag_detail_para_t 位布局一致） */
static void Parse_DetailPara(uint32_t para, AOA_Data_t *out)
{
    uint8_t mode  = (para >> 14) & 0x07;
    uint8_t recal = (para >> 17) & 0x01;
    uint8_t lock  = (para >> 18) & 0x01;

    out->battery_mv10 = (para >> 4) & 0x3FF;

    if (lock)              { out->car_mode = MODE_LOCK;   }
    else if (recal)        { out->car_mode = MODE_RECALL; }
    else if (mode == 1)    { out->car_mode = MODE_REMOTE; }
    else                   { out->car_mode = MODE_FOLLOW; }

    if (out->car_mode == MODE_REMOTE)
    {
        uint8_t up    = (para >> 10) & 0x01;
        uint8_t down  = (para >> 11) & 0x01;
        uint8_t left  = (para >> 12) & 0x01;
        uint8_t right = (para >> 13) & 0x01;

        if (up)         out->remote_cmd = RCSF;
        else if (down)  out->remote_cmd = BACKWARD;
        else if (left)  out->remote_cmd = SPOT_LEFT_TURN;
        else if (right) out->remote_cmd = SPOT_RIGHT_TURN;
        else            out->remote_cmd = STOP_MOTOR;
    }
}

bool APP_UWB_ParseFrame(const uint8_t *buf, uint16_t len, AOA_Data_t *out)
{
    uint16_t i;

    for (i = 0; i + 2 < len; i++)
    {
        if (buf[i] == UWB_CMD_SOP)
        {
            uint16_t msg_len = ((uint16_t)buf[i + 1] << 8) | buf[i + 2];

            if (msg_len == 0 || i + msg_len + 2 > len)
            {
                continue;
            }
            if (buf[i + msg_len + 1] != UWB_CMD_FOOT)
            {
                continue;
            }

            /* type 字段在 8+8 字节地址之后 */
            uint8_t type = buf[i + 3 + 16];
            if (type != 0x64 && type != 0x61)
            {
                continue;
            }

            const UWB_AoaPayload_t *p = (const UWB_AoaPayload_t *)&buf[i + 3 + 16 + 2];

            out->angle    = p->tag[0].angle;
            out->distance = p->tag[0].range;
            out->link_ok  = true;
            Parse_DetailPara(p->detail_para, out);
            return true;
        }
    }
    return false;
}
