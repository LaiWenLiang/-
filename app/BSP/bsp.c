#include "bsp.h"
#include "bsp_periph.h"
#include "bsp_motor.h"
#include "bsp_tof.h"
#include "bsp_oled.h"
#include "bsp_speaker.h"

/* BSP 层统一初始化（不含 UWB 串口——它依赖 FreeRTOS 队列，由 APP 层初始化） */
void BSP_Init(void)
{
    BSP_Periph_Init();

    /* 【无刷电机方案】有刷 PWM 驱动已停用。
       如需换回有刷电机：取消下行注释，并在 APP 层使用 PWM 版 Control_Execute。 */
    // BSP_Motor_Init();

    BSP_TOF_Init();
    BSP_OLED_Init();
    BSP_Speaker_Init();
    BSP_Iwdg_Init();
}
