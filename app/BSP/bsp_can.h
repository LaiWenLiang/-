#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "bsp.h"

/* CAN1: PB8(CAN_RX) / PB9(CAN_TX)，500Kbps，外接 TJA1050 收发器 */
/* 总线两端需各加 120Ω 终端电阻 */

typedef struct
{
    uint32_t id;       /* 11 位标准帧 ID */
    uint8_t  len;
    uint8_t  data[8];
} CanFrame_t;

void   BSP_CAN_Init(QueueHandle_t rx_queue);
uint8_t BSP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len);

#endif /* __BSP_CAN_H */
