#include "bsp_motor.h"

static void Motor_TIM_Config(TIM_TypeDef *TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period            = MOTOR_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC4Init(TIMx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIMx, ENABLE);
    TIM_Cmd(TIMx, ENABLE);
    TIM_CtrlPWMOutputs(TIMx, ENABLE);
}

void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    /* PA6/PA7 -> TIM3_CH1/CH2 ; PB0/PB1 -> TIM3_CH3/CH4 ; PB6~PB9 -> TIM4_CH1~CH4 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

    Motor_TIM_Config(TIM3);
    Motor_TIM_Config(TIM4);

    BSP_Motor_StopNow();
}

/* id: 1=A 2=B 3=C 4=D ; pwm: -MOTOR_ARR ~ +MOTOR_ARR（正负代表方向，H桥双PWM驱动） */
void BSP_Motor_SetPwm(uint8_t id, int pwm)
{
    uint32_t ch1, ch2;

    if (pwm > MOTOR_ARR)  pwm = MOTOR_ARR;
    if (pwm < -MOTOR_ARR) pwm = -MOTOR_ARR;

    if (pwm >= 0) { ch1 = MOTOR_ARR; ch2 = MOTOR_ARR - pwm; }
    else          { ch2 = MOTOR_ARR; ch1 = MOTOR_ARR + pwm; }

    switch (id)
    {
    case 1: PWMA1 = ch1; PWMA2 = ch2; break;
    case 2: PWMB1 = ch1; PWMB2 = ch2; break;
    case 3: PWMC1 = ch1; PWMC2 = ch2; break;
    case 4: PWMD1 = ch1; PWMD2 = ch2; break;
    default: break;
    }
}

void BSP_Motor_AllPwm(int a, int b, int c, int d)
{
    BSP_Motor_SetPwm(1, a);
    BSP_Motor_SetPwm(2, b);
    BSP_Motor_SetPwm(3, c);
    BSP_Motor_SetPwm(4, d);
}

void BSP_Motor_StopNow(void)
{
    BSP_Motor_AllPwm(0, 0, 0, 0);
}
