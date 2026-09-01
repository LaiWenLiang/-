#include "boot_jump.h"
#include "ota_config.h"
#include "stm32f4xx.h"

/***************************************************************************************************
 * 跳转 APP 前的检查：
 *   1. APP 第一个字 = 栈顶地址，必须在 SRAM 范围（0x20000000 开头）
 *   2. APP 第二个字 = 复位中断入口，必须在 APP 运行区范围内
 * 检查通过后：关总中断 -> 设置主栈指针 -> 重定位中断向量表 -> 跳转
 ***************************************************************************************************/
uint8_t Boot_JumpToApp(uint32_t app_addr)
{
    uint32_t stack_ptr;
    uint32_t reset_handler;
    void (*app_entry)(void);

    stack_ptr     = *(volatile uint32_t *)(app_addr);
    reset_handler = *(volatile uint32_t *)(app_addr + 4);

    /* 1. 栈顶地址检查（F407 SRAM 从 0x20000000 开始） */
    if ((stack_ptr & 0xFFF00000) != 0x20000000)
    {
        return 0;
    }

    /* 2. 复位入口检查 */
    if (reset_handler < app_addr || reset_handler > (OTA_APP_ADDR + OTA_APP_MAX_SIZE))
    {
        return 0;
    }

    /* 3. 关总中断（Bootloader 没用到的外设时钟不用管，内核干净最重要） */
    __disable_irq();

    /* 4. 关 SysTick，防止跳过去后先来一个莫名其妙的中断 */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 5. 设置主栈指针 + 重定位向量表 */
    __set_MSP(stack_ptr);
    SCB->VTOR = app_addr;

    /* 6. 跳转到 APP 的复位入口（这行之后不会再执行） */
    app_entry = (void (*)(void))reset_handler;
    app_entry();

    return 1;   /* 永远到不了这里 */
}
