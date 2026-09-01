#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "bsp.h"

#define MOTOR_ARR        5500   /* PWM 自动重装载值 */

/* PWM 通道宏：A=TIM3_CH3/CH4  B=TIM3_CH1/CH2  C=TIM4_CH1/CH2  D=TIM4_CH3/CH4 */
#define PWMA1   (TIM3->CCR3)   /* PB0 */
#define PWMA2   (TIM3->CCR4)   /* PB1 */
#define PWMB1   (TIM3->CCR1)   /* PA6 */
#define PWMB2   (TIM3->CCR2)   /* PA7 */
#define PWMC1   (TIM4->CCR1)   /* PB6 */
#define PWMC2   (TIM4->CCR2)   /* PB7 */
#define PWMD1   (TIM4->CCR3)   /* PB8 */
#define PWMD2   (TIM4->CCR4)   /* PB9 */

void BSP_Motor_Init(void);
void BSP_Motor_SetPwm(uint8_t id, int pwm);
void BSP_Motor_AllPwm(int a, int b, int c, int d);
void BSP_Motor_StopNow(void);

#endif /* __BSP_MOTOR_H */
