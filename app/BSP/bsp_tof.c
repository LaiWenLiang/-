#include "bsp_tof.h"
#include "bsp_i2c.h"
#include <math.h>

extern SemaphoreHandle_t g_mutex_i2c_tof;   /* 在 app_tasks.c 中创建 */

/* 简易卡尔曼（沿用原工程算法，按通道独立） */
static float Kalman_TOF(int distance, int group)
{
    static float   kalman_x[4], kalman_z[4], me[4];
    static uint8_t init[4];
    const float    est = 50.0f;
    float          k;

    if (init[group] == 0)
    {
        kalman_x[group] = distance;
        kalman_z[group] = distance;
        me[group]       = 0;
        init[group]     = 1;
        return kalman_x[group];
    }

    me[group]       = fabsf(kalman_z[group] - distance);
    k               = est / (est + me[group]);
    kalman_x[group] = kalman_x[group] + k * (distance - kalman_x[group]);
    kalman_z[group] = distance;
    return kalman_x[group];
}

static int TOF_Parse(int8_t read_ok, const uint8_t *buf)
{
    int d;

    if (read_ok != 0)
    {
        return TOF_DISTANCE_MAX;
    }
    d = (buf[0] << 8) | buf[1];
    if (d <= 0 || d >= TOF_DISTANCE_MAX)
    {
        d = TOF_DISTANCE_MAX;
    }
    return d;
}

void BSP_TOF_Init(void)
{
    BSP_I2C2_Init();
}

/* 读 4 路 TOF：持有互斥锁访问总线，读取失败/超量程记为 TOF_DISTANCE_MAX */
int8_t BSP_TOF_ReadAll(TOF_Distance_t *dist)
{
    uint8_t buf[2];
    int     raw;
    int8_t  ret = 0;

    if (xSemaphoreTake(g_mutex_i2c_tof, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return -1;
    }

    raw = TOF_Parse(BSP_I2C2_Read(TOF_L, TOF_REG_DISTANCE, buf, 2), buf);
    dist->left_mm = (raw >= TOF_DISTANCE_MAX) ? TOF_DISTANCE_MAX : (int)Kalman_TOF(raw, 0);

    raw = TOF_Parse(BSP_I2C2_Read(TOF_ML, TOF_REG_DISTANCE, buf, 2), buf);
    dist->middle_l_mm = (raw >= TOF_DISTANCE_MAX) ? TOF_DISTANCE_MAX : (int)Kalman_TOF(raw, 1);

    raw = TOF_Parse(BSP_I2C2_Read(TOF_MR, TOF_REG_DISTANCE, buf, 2), buf);
    dist->middle_r_mm = (raw >= TOF_DISTANCE_MAX) ? TOF_DISTANCE_MAX : (int)Kalman_TOF(raw, 2);

    raw = TOF_Parse(BSP_I2C2_Read(TOF_R, TOF_REG_DISTANCE, buf, 2), buf);
    dist->right_mm = (raw >= TOF_DISTANCE_MAX) ? TOF_DISTANCE_MAX : (int)Kalman_TOF(raw, 3);

    xSemaphoreGive(g_mutex_i2c_tof);
    return ret;
}
