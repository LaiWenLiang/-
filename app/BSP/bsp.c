#include "bsp.h"
#include "bsp_io.h"
#include "bsp_tof.h"
#include "bsp_oled.h"
#include "bsp_speaker.h"

/* BSP 层统一初始化（不含 UWB 串口——它依赖 FreeRTOS 队列，由 APP 层初始化） */
void BSP_Init(void)
{
    BSP_Periph_Init();
    BSP_TOF_Init();
    BSP_OLED_Init();
    BSP_Speaker_Init();
    BSP_Iwdg_Init();
}
