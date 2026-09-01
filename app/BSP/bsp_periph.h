#ifndef __BSP_PERIPH_H
#define __BSP_PERIPH_H

#include "bsp.h"

#define LED1_PIN    GPIO_Pin_9
#define LED2_PIN    GPIO_Pin_10
#define BEEP_PIN    GPIO_Pin_8
#define LED_GPIO    GPIOF
#define BEEP_GPIO   GPIOF

#define ON  0
#define OFF 1

#define LED1(a)  do { if(a) GPIO_SetBits(LED_GPIO, LED1_PIN);  else GPIO_ResetBits(LED_GPIO, LED1_PIN);  } while (0)
#define LED2(a)  do { if(a) GPIO_SetBits(LED_GPIO, LED2_PIN);  else GPIO_ResetBits(LED_GPIO, LED2_PIN);  } while (0)
#define BEEP(a)  do { if(a) GPIO_SetBits(BEEP_GPIO, BEEP_PIN); else GPIO_ResetBits(BEEP_GPIO, BEEP_PIN); } while (0)

void BSP_Periph_Init(void);
void BSP_Iwdg_Init(void);
void BSP_Iwdg_Feed(void);

#endif /* __BSP_PERIPH_H */
