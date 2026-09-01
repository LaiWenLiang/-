#ifndef __APP_UWB_H
#define __APP_UWB_H

#include "app.h"

/* UWB 协议帧：帧头 0x2A，帧尾 0x23（沿用原工程协议） */
#define UWB_CMD_SOP    0x2A
#define UWB_CMD_FOOT   0x23

bool APP_UWB_ParseFrame(const uint8_t *buf, uint16_t len, AOA_Data_t *out);

#endif /* __APP_UWB_H */
