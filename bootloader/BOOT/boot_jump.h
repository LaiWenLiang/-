#ifndef __BOOT_JUMP_H
#define __BOOT_JUMP_H

#include <stdint.h>

/* 跳转到 APP。返回 0 表示校验失败没跳（正常情况下跳走就不会返回了） */
uint8_t Boot_JumpToApp(uint32_t app_addr);

#endif /* __BOOT_JUMP_H */
