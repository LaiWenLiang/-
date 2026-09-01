#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp.h"
#include "app.h"
#include "app_tasks.h"

int main(void)
{
    /* APP 运行在 0x08020000（Bootloader 之后），中断向量表必须重定位 */
    SCB->VTOR = 0x08020000;

    /* FreeRTOS requires all 4 NVIC priority bits to be preemption priority. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Init SysTick at HCLK for the OS-aware delay + FreeRTOS tick. */
    delay_init(168);

    /* 底层硬件：LED/蜂鸣器/TOF/OLED/语音/看门狗（电机走 CAN，由 APP 层初始化） */
    BSP_Init();

    /* 创建队列/互斥锁/事件组/任务，并初始化 UWB 串口 */
    APP_Tasks_Create();

    vTaskStartScheduler();

    /* vTaskStartScheduler() never returns. */
    while (1)
    {
    }
}
