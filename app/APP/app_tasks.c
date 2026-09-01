#include "app_tasks.h"
#include <limits.h>
#include "app_control.h"
#include "app_avoid.h"
#include "app_uwb.h"
#include "app_filter.h"
#include "app_speedloop.h"
#include "usart.h"
#include "bsp_tof.h"
#include "bsp_oled.h"
#include "bsp_speaker.h"
#include "bsp_periph.h"
#include "bsp_can.h"
#include "bsp_oidmotor.h"
#include "bsp_key.h"
#include "bsp_bat.h"
#include "app_debug.h"
#include "app_power.h"
#include "app_ota.h"
#include "app_onenet_ota.h"
#include "ota_config.h"
#include "timers.h"

/***************************************************************************************************
 * 全局 RTOS 对象
 ***************************************************************************************************/
QueueHandle_t      g_queue_uwb_rx   = NULL;
QueueHandle_t      g_queue_aoa      = NULL;
QueueHandle_t      g_queue_tof      = NULL;
QueueHandle_t      g_queue_can_rx   = NULL;
QueueHandle_t      g_queue_ota_rx   = NULL;
SemaphoreHandle_t  g_mutex_i2c_tof  = NULL;
EventGroupHandle_t g_event_system   = NULL;
TaskHandle_t       g_task_safety    = NULL;
TaskHandle_t       g_task_control   = NULL;
TaskHandle_t       g_task_speaker   = NULL;

KalmanState_t g_kf_uwb_angle;
KalmanState_t g_kf_uwb_dist;
int32_t       g_target_erpm_l = 0;
int32_t       g_target_erpm_r = 0;

uint8_t g_ota_state = OTA_ST_IDLE;    /* OTA 人工确认状态 */
char    g_ota_new_ver[16] = {0};      /* OneNET 上发现的新版本号 */

#define UWB_LINK_TIMEOUT_MS   1000   /* UWB 失联判定时间 */

/***************************************************************************************************
 * Task_UWB_Parse（中优先级，队列唤醒）：解析 UWB 帧 -> 覆盖式写入 AOA 队列
 ***************************************************************************************************/
static void Task_UWB_Parse(void *param)
{
    UWB_Frame_t frame;
    AOA_Data_t  aoa;

    Kalman_Init(&g_kf_uwb_angle, 0.05f, 1.0f);
    Kalman_Init(&g_kf_uwb_dist,  0.05f, 1.0f);

    for (;;)
    {
        if (xQueueReceive(g_queue_uwb_rx, &frame, portMAX_DELAY) == pdTRUE)
        {
            if (APP_UWB_ParseFrame(frame.buf, frame.len, &aoa))
            {
                Power_FeedAlive();   /* UWB 有数据 = 有活动 */
                aoa.angle    = (int)Kalman_Update(&g_kf_uwb_angle, aoa.angle);
                aoa.distance = (int)Kalman_Update(&g_kf_uwb_dist,  aoa.distance);
                xQueueOverwrite(g_queue_aoa, &aoa);   /* 只保留最新一帧 */
                xEventGroupSetBits(g_event_system, EVT_UWB_LINK_OK);
            }
        }
    }
}

/***************************************************************************************************
 * Task_Safety（最高优先级，20ms）：TOF 读取 + 多级避障 + 急停事件 + 电池电压检测
 ***************************************************************************************************/
#define BAT_WARN_HOLD_MS    5000    /* 低于告警门限持续 5 秒才播报 */
#define BAT_CRIT_HOLD_MS    3000    /* 低于严重门限持续 3 秒才停车 */
#define BAT_VOICE_GAP_MS    60000   /* 低电播报最小间隔 60 秒 */

static void Task_Safety(void *param)
{
    TOF_Distance_t  tof;
    TurnDirection_t avoid;
    TickType_t      last = xTaskGetTickCount();
    TickType_t      low_since  = 0;   /* 低于告警门限的起始时刻，0=正常 */
    TickType_t      crit_since = 0;   /* 低于严重门限的起始时刻 */
    TickType_t      last_voice = 0;   /* 上次播报时刻 */
    uint32_t        bat_mv;

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));

        /* ---- 电池电压检测 ---- */
        Bat_Update();
        bat_mv = Bat_GetVoltageMv();

        if (bat_mv > BAT_VOLT_WARN)
        {
            low_since = 0;
        }
        else if (bat_mv > 30000)   /* 30V 以下视为没接电池/探头悬空，不误报 */
        {
            if (low_since == 0)
            {
                low_since = xTaskGetTickCount();
            }
            /* 持续低电 5 秒，且距上次播报超过 60 秒 -> 播报 */
            else if ((xTaskGetTickCount() - low_since) > pdMS_TO_TICKS(BAT_WARN_HOLD_MS) &&
                     (xTaskGetTickCount() - last_voice) > pdMS_TO_TICKS(BAT_VOICE_GAP_MS))
            {
                last_voice = xTaskGetTickCount();
                xTaskNotify(g_task_speaker, VOICE_CAR_LOW_BATTERY, eSetValueWithOverwrite);
            }
        }

        if (bat_mv > BAT_VOLT_CRIT || bat_mv <= 30000)
        {
            crit_since = 0;
            if (bat_mv > BAT_VOLT_CRIT + 1000)   /* 回差 1V：恢复到 41V 以上才解除停车 */
            {
                xEventGroupClearBits(g_event_system, EVT_BATTERY_LOW);
            }
        }
        else
        {
            if (crit_since == 0)
            {
                crit_since = xTaskGetTickCount();
            }
            else if ((xTaskGetTickCount() - crit_since) > pdMS_TO_TICKS(BAT_CRIT_HOLD_MS))
            {
                xEventGroupSetBits(g_event_system, EVT_BATTERY_LOW);   /* 严重低电：强制停车 */
            }
        }

        /* ---- TOF 避障 ---- */
        if (BSP_TOF_ReadAll(&tof) != 0)
        {
            continue;   /* 总线忙，本周期跳过 */
        }

        xQueueOverwrite(g_queue_tof, &tof);

        avoid = Avoid_Judge(&tof);
        if (avoid == STOP_MOTOR)
        {
            xEventGroupSetBits(g_event_system, EVT_TOF_EMERGENCY);
        }
        else if (avoid != NO_ACTION)
        {
            xEventGroupSetBits(g_event_system, EVT_OBSTACLE_WARN);
            /* 警戒动作也直接通知控制任务，通知值 = 避障指令 */
            xTaskNotify(g_task_control, (uint32_t)avoid, eSetValueWithOverwrite);
        }
        else
        {
            xEventGroupClearBits(g_event_system, EVT_TOF_EMERGENCY | EVT_OBSTACLE_WARN);
        }
    }
}

/***************************************************************************************************
 * Task_Control（高优先级，20ms）：模式决策 + 电机输出；急停事件可立即抢占
 ***************************************************************************************************/
static void Task_Control(void *param)
{
    AOA_Data_t      aoa = {0};
    uint16_t        speed = 0;
    uint32_t        notify_val;
    TickType_t      last = xTaskGetTickCount();
    CarMode_t       last_mode = MODE_NONE;

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));

        /* 急停最高优先：再生制动主动刹停 */
        if (xEventGroupGetBits(g_event_system) & EVT_TOF_EMERGENCY)
        {
            Control_EmergencyBrake();
            continue;
        }

        /* 严重低电：停车保护电池（仅次于急停） */
        if (xEventGroupGetBits(g_event_system) & EVT_BATTERY_LOW)
        {
            Control_Execute(STOP_MOTOR, 0);
            continue;
        }

        /* 省电态：保持停车（优先级同 OTA） */
        if (xEventGroupGetBits(g_event_system) & EVT_POWER_SAVE)
        {
            Control_Execute(STOP_MOTOR, 0);
            continue;
        }

        /* OTA 升级中：保持停车，暂停一切跟随/遥控逻辑 */
        if (xEventGroupGetBits(g_event_system) & EVT_OTA_MODE)
        {
            Control_Execute(STOP_MOTOR, 0);
            continue;
        }

        /* 失联保护：速度环目标清零，主动停稳 */
        if ((xEventGroupGetBits(g_event_system) & EVT_UWB_LINK_OK) == 0)
        {
            Control_Execute(STOP_MOTOR, 0);
            continue;
        }

        /* 警戒避障指令（任务通知直达） */
        if (xTaskNotifyWait(0, 0, &notify_val, 0) == pdTRUE && notify_val != NO_ACTION)
        {
            Control_Execute((TurnDirection_t)notify_val, speed);
            continue;
        }

        /* 正常模式决策 */
        if (xQueuePeek(g_queue_aoa, &aoa, 0) == pdTRUE)
        {
            if (aoa.car_mode != last_mode)
            {
                last_mode = (CarMode_t)aoa.car_mode;
                xTaskNotify(g_task_speaker,
                            (aoa.car_mode == MODE_REMOTE) ? VOICE_REMOTE_MODE : VOICE_FOLLOW_MODE,
                            eSetValueWithOverwrite);
            }

            switch (aoa.car_mode)
            {
            case MODE_FOLLOW:
                Control_FollowStep(&aoa);   /* 串级控制：距离环+角度环+速度环 */
                break;

            case MODE_REMOTE:
                Control_Execute((TurnDirection_t)aoa.remote_cmd, REMOTE_SPEED);
                break;

            case MODE_LOCK:
            default:
                Control_Execute(STOP_MOTOR, 0);
                break;
            }
        }
    }
}

/***************************************************************************************************
 * Task_Display（低优先级，200ms）：OLED 刷新 + 按键扫描
 *  KEY1/KEY2：升级提示出现时 = 接受/拒绝
 *  KEY3    ：平时 = 切换 USART1 蓝牙/RS485 模式
 ***************************************************************************************************/
static void Task_Display(void *param)
{
    TOF_Distance_t tof;
    char           line[24];
    TickType_t     last = xTaskGetTickCount();
    uint8_t        key;
    static uint8_t last_ota_state = OTA_ST_IDLE;

    BSP_OLED_Clear();
    BSP_OLED_ShowString(0, 0, "FOLLOW CAR F407");

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(200));

        key = Key_Scan();

        /* 按键 = 有活动（省电态下按键就是唤醒信号） */
        if (key != KEY_NONE)
        {
            Power_FeedAlive();
        }

        /* 省电态：OLED 已断电，只留按键扫描用于唤醒 */
        if (xEventGroupGetBits(g_event_system) & EVT_POWER_SAVE)
        {
            continue;
        }

        /* KEY3：切换 USART1 模式（升级提示出现时无效） */
        if (key == KEY_3 && g_ota_state != OTA_ST_ASK && g_ota_state != OTA_ST_UPGRADING)
        {
            g_uart1_mode = (g_uart1_mode == UART1_MODE_BT) ? UART1_MODE_485 : UART1_MODE_BT;
            Debug_Printf("uart1 mode -> %s\n", (g_uart1_mode == UART1_MODE_BT) ? "BT" : "RS485");
        }

        /* 升级提示时的按键选择 */
        if (g_ota_state == OTA_ST_ASK)
        {
            if (key == KEY_1) g_ota_state = OTA_ST_ACCEPT;
            if (key == KEY_2) g_ota_state = OTA_ST_REJECT;
        }

        /* 升级中：整屏只显示"正在升级中" */
        if (g_ota_state == OTA_ST_UPGRADING)
        {
            if (last_ota_state != OTA_ST_UPGRADING)
            {
                BSP_OLED_Clear();
                BSP_OLED_ShowString(0, 2, "OTA UPGRADING...");
                BSP_OLED_ShowString(0, 4, "DO NOT POWER OFF");
            }
            last_ota_state = g_ota_state;
            continue;
        }

        /* 发现新固件：提示等待按键 */
        if (g_ota_state == OTA_ST_ASK)
        {
            snprintf(line, sizeof(line), "NEW FW: %s", g_ota_new_ver);
            BSP_OLED_ShowString(0, 4, line);
            BSP_OLED_ShowString(0, 6, "K1:OK    K2:NO ");
        }
        else
        {
            BSP_OLED_ShowString(0, 4, "                ");
            BSP_OLED_ShowString(0, 6, (g_uart1_mode == UART1_MODE_BT) ?
                                     "MODE: BT(VOFA)   " : "MODE: RS485(OTA) ");
        }
        last_ota_state = g_ota_state;

        if (xQueuePeek(g_queue_tof, &tof, 0) == pdTRUE)
        {
            snprintf(line, sizeof(line), "L:%3d ML:%3d", tof.left_mm, tof.middle_l_mm);
            BSP_OLED_ShowString(0, 2, line);
            snprintf(line, sizeof(line), "R:%3d MR:%3d", tof.right_mm, tof.middle_r_mm);
            BSP_OLED_ShowString(0, 3, line);
        }

        /* 电池电压 + 电量百分比 */
        {
            uint32_t mv = Bat_GetVoltageMv();
            snprintf(line, sizeof(line), "BAT:%lu.%luV %3u%% ",
                     mv / 1000, (mv % 1000) / 100, Bat_GetPercent());
            BSP_OLED_ShowString(0, 5, line);
        }

        if (xEventGroupGetBits(g_event_system) & (EVT_TOF_EMERGENCY | EVT_OBSTACLE_WARN))
        {
            LED1(ON);
        }
        else
        {
            LED1(OFF);
        }
    }
}

/***************************************************************************************************
 * Task_Speaker（低优先级，任务通知触发）：语音播报
 ***************************************************************************************************/
static void Task_Speaker(void *param)
{
    uint32_t voice_id;

    for (;;)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &voice_id, portMAX_DELAY) == pdTRUE)
        {
            BSP_Speaker_Play((VoiceId_t)voice_id);
        }
    }
}

/***************************************************************************************************
 * Task_MotorCan（中高优先级）：
 *   ① 200ms 周期向两个驱动器发心跳（断心跳驱动器会放空电机！）
 *   ② 每 20ms 轮流查询左右轮转速（速度环反馈来源）
 *   ③ 解析 CAN 接收队列里的驱动器应答帧
 ***************************************************************************************************/
#define MOTOR_HEARTBEAT_MS   200   /* 应为驱动器超时时间的 1/2（默认超时 1000ms） */

static void Task_MotorCan(void *param)
{
    CanFrame_t  frame;
    TickType_t  last = xTaskGetTickCount();
    uint8_t     poll_id = OID_ID_LEFT;
    uint32_t    beat_div = 0;

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));

        /* 省电态下不发电流/心跳，驱动器自动放空省电 */
        if ((xEventGroupGetBits(g_event_system) & EVT_POWER_SAVE) == 0)
        {
            /* 心跳：每 MOTOR_HEARTBEAT_MS 发一次 */
            if (++beat_div >= (MOTOR_HEARTBEAT_MS / 20))
            {
                beat_div = 0;
                OID_Heartbeat(OID_ID_LEFT);
                OID_Heartbeat(OID_ID_RIGHT);
            }

            /* 轮流查询转速反馈 */
            OID_QuerySpeed(poll_id);
            poll_id = (poll_id == OID_ID_LEFT) ? OID_ID_RIGHT : OID_ID_LEFT;
        }

        /* 解析本周期收到的所有应答帧（CAN 总线只跑电机驱动器） */
        while (xQueueReceive(g_queue_can_rx, &frame, 0) == pdTRUE)
        {
            OID_ParseFrame(&frame);
        }
    }
}

/***************************************************************************************************
 * Task_Debug（低优先级，50ms）：USART1/VOFA 命令解析 + 波形输出
 ***************************************************************************************************/
static void Task_Debug(void *param)
{
    TickType_t last = xTaskGetTickCount();

    APP_Debug_Init(115200);

    for (;;)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
        APP_Debug_Process();
        APP_Debug_Telemetry();
    }
}

/***************************************************************************************************
 * 软件定时器（500ms）：喂狗 + UWB 失联检测
 ***************************************************************************************************/
static void Timer_Monitor(TimerHandle_t timer)
{
    static uint8_t heartbeat = 0;

    BSP_Iwdg_Feed();
    heartbeat = !heartbeat;
    LED2(heartbeat);   /* 心跳灯 */

    if ((xTaskGetTickCount() - UWB_UART_LastRxTick()) > pdMS_TO_TICKS(UWB_LINK_TIMEOUT_MS))
    {
        xEventGroupClearBits(g_event_system, EVT_UWB_LINK_OK);
    }
}

/***************************************************************************************************
 * APP 层入口：创建所有 RTOS 对象与任务
 ***************************************************************************************************/
void APP_Tasks_Create(void)
{
    /* 队列 */
    g_queue_uwb_rx = xQueueCreate(4, sizeof(UWB_Frame_t));
    g_queue_aoa    = xQueueCreate(1, sizeof(AOA_Data_t));
    g_queue_tof    = xQueueCreate(1, sizeof(TOF_Distance_t));
    g_queue_can_rx = xQueueCreate(8, sizeof(CanFrame_t));
    g_queue_ota_rx = xQueueCreate(4, sizeof(OtaFrame_t));   /* RS485 OTA 帧（帧较大，队列给浅一点） */

    /* 互斥锁与事件组 */
    g_mutex_i2c_tof = xSemaphoreCreateMutex();
    g_event_system  = xEventGroupCreate();

    configASSERT(g_queue_uwb_rx && g_queue_aoa && g_queue_tof && g_queue_can_rx &&
                 g_queue_ota_rx && g_mutex_i2c_tof && g_event_system);

    /* 任务：数值越大优先级越高 */
    xTaskCreate(Task_Safety,   "safety",   256, NULL, 5, &g_task_safety);
    xTaskCreate(Task_Control,  "control",  384, NULL, 4, &g_task_control);
    xTaskCreate(Task_UWB_Parse,"uwb",      384, NULL, 3, NULL);
    xTaskCreate(Task_MotorCan, "motorcan", 256, NULL, 3, NULL);
    xTaskCreate(Task_Display,  "display",  256, NULL, 2, NULL);
    xTaskCreate(Task_Speaker,  "speaker",  192, NULL, 1, &g_task_speaker);
    xTaskCreate(Task_Debug,    "debug",    384, NULL, 1, NULL);
    xTaskCreate(App_Ota_Task,  "ota",      384, NULL, 1, NULL);
    xTaskCreate(App_OnenetOta_Task, "onenet", 1024, NULL, 1, NULL);   /* HTTP 字符串较多，栈给大 */
    xTaskCreate(App_Power_Task,   "power",   192, NULL, 1, NULL);

    /* 监控软件定时器 */
    xTimerStart(xTimerCreate("monitor", pdMS_TO_TICKS(500), pdTRUE, NULL, Timer_Monitor), 0);

    /* 依赖队列的通信外设在这里初始化 */
    UWB_UART_Init(115200, g_queue_uwb_rx);
    BSP_CAN_Init(g_queue_can_rx);
    Debug_UART_SetOtaQueue(g_queue_ota_rx);   /* USART1 RS485 模式 -> OTA 队列 */
    BSP_Key_Init();
    BSP_Bat_Init();   /* 电池电压 ADC（PA5） */
    App_Ota_Init();   /* W25Q32 / W24C02 */
}
