#include "bsp_key.h"

void BSP_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = KEY1_PIN | KEY2_PIN | KEY3_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;   /* 上拉，按下为低电平 */
    GPIO_Init(KEY_GPIO, &GPIO_InitStructure);
}

/* 简单消抖：连续 2 次读到同一按键按下才算有效，松手后复位 */
uint8_t Key_Scan(void)
{
    static uint8_t last_key = KEY_NONE;   /* 上一次确认时的原始键值 */
    static uint8_t press_cnt = 0;
    uint8_t now = KEY_NONE;

    if (GPIO_ReadInputDataBit(KEY_GPIO, KEY1_PIN) == Bit_RESET) now = KEY_1;
    else if (GPIO_ReadInputDataBit(KEY_GPIO, KEY2_PIN) == Bit_RESET) now = KEY_2;
    else if (GPIO_ReadInputDataBit(KEY_GPIO, KEY3_PIN) == Bit_RESET) now = KEY_3;

    if (now == KEY_NONE)
    {
        last_key  = KEY_NONE;   /* 已松手，允许下一次触发 */
        press_cnt = 0;
        return KEY_NONE;
    }

    if (now == last_key)
        return KEY_NONE;        /* 同一个键还按着，不重复触发 */

    press_cnt++;
    if (press_cnt >= 2)         /* 连续 2 次扫描都按下（调用周期 200ms 时约 0.4s 内有效） */
    {
        press_cnt = 0;
        last_key  = now;
        return now;
    }
    return KEY_NONE;
}
