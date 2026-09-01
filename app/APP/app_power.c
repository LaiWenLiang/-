#include "app_power.h"
#include "app.h"
#include "bsp_oled.h"
#include "bsp_esp8266.h"
#include "usart.h"

static TickType_t s_last_active = 0;   /* 最后一次活动的时刻 */

void Power_FeedAlive(void)
{
    s_last_active = xTaskGetTickCount();
}

/* 进入省电态：控制任务看到 EVT_POWER_SAVE 会停车，心跳任务停发，OLED/WiFi 断电 */
static void Power_EnterSave(void)
{
    xEventGroupSetBits(g_event_system, EVT_POWER_SAVE);
    BSP_OLED_DisplayOnOff(0);
    ESP8266_PowerDown();
    Debug_Printf("power: enter save mode\n");
}

/* 退出省电态：OLED 亮回来，ESP8266 重新上电（onenet 任务会自动重连 WiFi） */
static void Power_ExitSave(void)
{
    ESP8266_PowerUp();
    BSP_OLED_DisplayOnOff(1);
    xEventGroupClearBits(g_event_system, EVT_POWER_SAVE);
    Debug_Printf("power: wake up\n");
}

void App_Power_Task(void *param)
{
    TickType_t entered_tick = 0;   /* 进入省电态时记录的活动时刻，变了就说明有新活动 */

    (void)param;
    s_last_active = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if ((xEventGroupGetBits(g_event_system) & EVT_POWER_SAVE) == 0)
        {
            /* 正常态：空闲超时且不在升级流程中 -> 进省电态 */
            if ((xTaskGetTickCount() - s_last_active) > pdMS_TO_TICKS(POWER_SAVE_TIMEOUT_MS) &&
                g_ota_state == OTA_ST_IDLE &&
                (xEventGroupGetBits(g_event_system) & EVT_OTA_MODE) == 0)
            {
                entered_tick = s_last_active;
                Power_EnterSave();
            }
        }
        else
        {
            /* 省电态：活动时刻变了（UWB 来数据 / 按了键）-> 唤醒 */
            if (s_last_active != entered_tick)
            {
                Power_ExitSave();
            }
        }
    }
}
