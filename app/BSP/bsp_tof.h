#ifndef __BSP_TOF_H
#define __BSP_TOF_H

#include "bsp.h"

/* 4 个 TOF250 的 I2C 设定地址（同 F103 工程） */
#define TOF_R   0x04
#define TOF_MR  0x18
#define TOF_ML  0x2C
#define TOF_L   0x40

#define TOF_REG_DISTANCE   0x00
#define TOF_DISTANCE_MAX   100   /* cm，读取失败/超量程统一为该值 */

typedef struct
{
    int left_mm;
    int middle_l_mm;
    int middle_r_mm;
    int right_mm;
} TOF_Distance_t;

void   BSP_TOF_Init(void);
int8_t BSP_TOF_ReadAll(TOF_Distance_t *dist);

#endif /* __BSP_TOF_H */
