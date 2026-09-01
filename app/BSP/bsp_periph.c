#include "bsp_periph.h"

void BSP_Periph_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = LED1_PIN | LED2_PIN | BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(LED_GPIO, &GPIO_InitStructure);

    LED1(OFF);
    LED2(OFF);
    BEEP(OFF);
}

/* 独立看门狗：约 1s 超时 */
void BSP_Iwdg_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_SetReload(1000);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void BSP_Iwdg_Feed(void)
{
    IWDG_ReloadCounter();
}
