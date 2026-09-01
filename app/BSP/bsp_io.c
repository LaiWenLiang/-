#include "bsp_io.h"

/***************************************************************************************************
 * LED / 蜂鸣器 / 看门狗
 ***************************************************************************************************/
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

/***************************************************************************************************
 * 按键（PA0/PA1/PA4，低电平触发）
 ***************************************************************************************************/
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

/***************************************************************************************************
 * 电池电压检测（ADC1_IN5 / PA5，轮询单次转换）
 * 每 20ms 采一次，16 次滑动平均滤波（电机大电流时母线电压会抖），
 * 换算成 mV 后用电压查表法估算电量百分比。
 ***************************************************************************************************/
#define BAT_AVG_NUM   16   /* 滑动平均点数 */

static uint32_t s_volt_mv = 0;    /* 滤波后的电池电压 mV */
static uint8_t  s_percent = 0;
static uint16_t s_avg_buf[BAT_AVG_NUM];
static uint8_t  s_avg_pos = 0;
static uint8_t  s_avg_cnt = 0;    /* 上电初期缓冲还没填满 */

void BSP_Bat_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef  ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;   /* 模拟输入 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_144Cycles);
    ADC_Cmd(ADC1, ENABLE);
}

/* 单次 ADC 采样（轮询，144 周期采样时间，约十几微秒） */
static uint16_t Bat_ReadAdc(void)
{
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
    }
    return ADC_GetConversionValue(ADC1);
}

/* 13S 三元锂 电压 -> 电量百分比（静置电压粗略查表） */
static uint8_t Bat_VoltToPercent(uint32_t mv)
{
    if (mv >= 54600) return 100;
    if (mv >= 50700) return (uint8_t)(70 + (mv - 50700) * 30 / 3900);   /* 50.7~54.6V -> 70~100% */
    if (mv >= 48100) return (uint8_t)(50 + (mv - 48100) * 20 / 2600);   /* 48.1~50.7V -> 50~70% */
    if (mv >= 46000) return (uint8_t)(30 + (mv - 46000) * 20 / 2100);   /* 46.0~48.1V -> 30~50% */
    if (mv >= 42900) return (uint8_t)(10 + (mv - 42900) * 20 / 3100);   /* 42.9~46.0V -> 10~30% */
    if (mv >= 39000) return (uint8_t)((mv - 39000) * 10 / 3900);        /* 39.0~42.9V -> 0~10% */
    return 0;
}

void Bat_Update(void)
{
    uint32_t sum = 0;
    uint8_t  i;

    s_avg_buf[s_avg_pos] = Bat_ReadAdc();
    s_avg_pos = (s_avg_pos + 1) % BAT_AVG_NUM;
    if (s_avg_cnt < BAT_AVG_NUM)
    {
        s_avg_cnt++;
    }

    for (i = 0; i < s_avg_cnt; i++)
    {
        sum += s_avg_buf[i];
    }

    /* V_bat(mV) = ADC * 3300 / 4096 * 分压比19 */
    s_volt_mv = (sum / s_avg_cnt) * 3300UL / 4096UL * BAT_DIV_RATIO;
    s_percent = Bat_VoltToPercent(s_volt_mv);
}

uint32_t Bat_GetVoltageMv(void)
{
    return s_volt_mv;
}

uint8_t Bat_GetPercent(void)
{
    return s_percent;
}
