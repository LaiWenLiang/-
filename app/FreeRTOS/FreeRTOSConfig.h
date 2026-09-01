/*
 * FreeRTOS V202212.01
 * FreeRTOSConfig.h - STM32F407 (Cortex-M4F) template configuration
 *
 * Ported from the official FreeRTOS distribution.  SysTick is owned by
 * SYSTEM/delay/delay.c (ALIENTEK OS-aware delay), which forwards the tick to
 * xPortSysTickHandler().  Therefore only SVC and PendSV are name-mapped below.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
extern uint32_t SystemCoreClock;

/*-----------------------------------------------------------
 * Scheduler / kernel options.
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

#define configCPU_CLOCK_HZ                      ( SystemCoreClock )          /* 168 MHz on F407 */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )     /* 1 ms tick */
#define configMAX_PRIORITIES                    ( 32 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 20 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/*-----------------------------------------------------------
 * Kernel feature includes.
 *----------------------------------------------------------*/
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    1
#define configUSE_TASK_NOTIFICATIONS            1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/*-----------------------------------------------------------
 * Software timer options.
 *----------------------------------------------------------*/
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/*-----------------------------------------------------------
 * Co-routine options (not used).
 *----------------------------------------------------------*/
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/*-----------------------------------------------------------
 * API functions to include.
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1

/*-----------------------------------------------------------
 * Interrupt priority (Cortex-M4F has 4 priority bits).
 * configLIBRARY_* use the ST library convention (0..15), then they are
 * shifted into the raw NVIC register format (0..255) below.
 *----------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS        __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS        4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*-----------------------------------------------------------
 * Cortex-M handler name mapping.
 *
 * SVC and PendSV are defined inside port.c as vPortSVCHandler and
 * xPortPendSVHandler, so they are mapped onto the vector-table names here.
 *
 * SysTick is intentionally NOT mapped: SYSTEM/delay/delay.c defines
 * SysTick_Handler and calls xPortSysTickHandler() from it, so mapping it here
 * would create a duplicate definition.
 *----------------------------------------------------------*/
#define xPortPendSVHandler  PendSV_Handler
#define vPortSVCHandler     SVC_Handler

/*-----------------------------------------------------------
 * Assertion: spin forever on failure (no stdio dependency).
 *----------------------------------------------------------*/
#define configASSERT( x )   if( ( x ) == 0 ) { for( ;; ); }

#endif /* FREERTOS_CONFIG_H */
