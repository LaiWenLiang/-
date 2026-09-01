#ifndef __APP_H
#define __APP_H

#include "bsp.h"

/***************************************************************************************************
 * 车辆模式与运动指令（沿用原工程定义）
 ***************************************************************************************************/
typedef enum
{
    NO_ACTION = 0,
    RCSF = 1,        /* 遥控直行 */
    FOLLOW_STRAIGHT, /* 跟随直行 */
    STOP_MOTOR,      /* 停止 */
    BACKWARD,        /* 后退 */
    SPOT_LEFT_TURN,  /* 原地左转 */
    SPOT_RIGHT_TURN, /* 原地右转 */
    LEFT_TURN,
    RIGHT_TURN,
    FOLLOW_CAR,
    LEFT_BACKWARD,   /* 左倒车 */
    RIGHT_BACKWARD,  /* 右倒车 */
} TurnDirection_t;

typedef enum
{
    MODE_NONE = 0,
    MODE_LOCK,     /* 锁定 */
    MODE_FOLLOW,   /* 跟随 */
    MODE_RECALL,   /* 召回 */
    MODE_REMOTE,   /* 遥控 */
} CarMode_t;

/***************************************************************************************************
 * UWB 解析后的定位 + 遥控数据
 ***************************************************************************************************/
typedef struct
{
    int      angle;          /* 目标角度(°) */
    int      distance;       /* 目标距离(cm) */
    uint8_t  remote_cmd;     /* 遥控按键指令 */
    uint8_t  car_mode;       /* 当前模式 */
    uint16_t battery_mv10;   /* 电池电压(0.01V) */
    bool     link_ok;        /* 数据有效标志 */
} AOA_Data_t;

/***************************************************************************************************
 * 系统事件组位定义
 ***************************************************************************************************/
#define EVT_UWB_LINK_OK     (1 << 0)
#define EVT_TOF_EMERGENCY   (1 << 1)
#define EVT_BATTERY_LOW     (1 << 2)
#define EVT_OBSTACLE_WARN   (1 << 3)
#define EVT_REMOTE_ACTIVE   (1 << 4)
#define EVT_OTA_MODE        (1 << 5)   /* OTA 升级中：控制任务保持停车 */

/* 全局 RTOS 对象（app_tasks.c 创建） */
extern QueueHandle_t      g_queue_uwb_rx;    /* UWB 原始帧 */
extern QueueHandle_t      g_queue_aoa;       /* 解析后定位数据（长度1，覆盖式） */
extern QueueHandle_t      g_queue_tof;       /* TOF 距离数据 */
extern QueueHandle_t      g_queue_can_rx;    /* CAN 接收帧（驱动器应答） */
extern QueueHandle_t      g_queue_ota_rx;    /* RS485 OTA 升级帧（OtaFrame_t，USART1 中断投递） */
extern SemaphoreHandle_t  g_mutex_i2c_tof;   /* TOF 总线互斥锁 */
extern EventGroupHandle_t g_event_system;    /* 系统事件组 */
extern TaskHandle_t       g_task_safety;
extern TaskHandle_t       g_task_control;
extern TaskHandle_t       g_task_speaker;

/* 调试层可观测/可调的全局量 */
typedef struct
{
    float x;   /* 状态估计值 */
    float p;   /* 估计误差协方差 */
    float q;   /* 过程噪声 */
    float r;   /* 测量噪声 */
} KalmanState_t;

extern KalmanState_t g_kf_uwb_angle;   /* UWB 角度卡尔曼（VOFA 可调 Q/R） */
extern KalmanState_t g_kf_uwb_dist;    /* UWB 距离卡尔曼 */
extern int32_t       g_target_erpm_l;  /* 左轮目标转速（波形显示用） */
extern int32_t       g_target_erpm_r;  /* 右轮目标转速 */

/* OTA 人工确认状态（onenet 任务置位，display 任务显示 + 按键处理） */
#define OTA_ST_IDLE        0
#define OTA_ST_ASK         1   /* 发现新固件，OLED 提示，等待 KEY1/KEY2 */
#define OTA_ST_ACCEPT      2   /* 用户已接受（KEY1） */
#define OTA_ST_REJECT      3   /* 用户已拒绝（KEY2） */
#define OTA_ST_UPGRADING   4   /* 正在升级中 */
extern uint8_t g_ota_state;
extern char    g_ota_new_ver[16];

void APP_Init(void);

#endif /* __APP_H */
