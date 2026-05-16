#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f10x.h"
#include <stdint.h>

/*-----------------------------------------------------------
 * 基本调度配置
 *----------------------------------------------------------*/

/* 使用抢占式调度 */
#define configUSE_PREEMPTION                    1

/* 使用空闲任务Hook：阶段一可先关闭 */
#define configUSE_IDLE_HOOK                     0

/* 使用Tick Hook：阶段一关闭 */
#define configUSE_TICK_HOOK                     0

/* STM32F103C8T6 主频 72 MHz */
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 72000000 )

/* FreeRTOS系统节拍 1000 Hz，即 1 tick = 1 ms */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )

/* 支持多个任务优先级
 * 后续规划：
 * ControlTask   高
 * CommTask      中高
 * KeyTask       中
 * DisplayTask   低
 */
#define configMAX_PRIORITIES                    6

/* 最小任务栈深度
 * 单位是 StackType_t，不一定直接等于字节。
 * Cortex-M3 下通常按 32-bit word 计。
 */
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )

/* FreeRTOS 动态堆大小
 * STM32F103C8T6 RAM 20 KB。
 * 当前项目已有 OLED 显存、串口缓存、PID变量、全局数据等，
 * 阶段一建议先使用 6 KB，属于较保守起点。
 * 后续创建任务后，再通过 xPortGetFreeHeapSize() 观察剩余堆。
 */
#define configTOTAL_HEAP_SIZE    ( ( size_t ) ( 10 * 1024 ) )

/* 任务名称最大长度 */
#define configMAX_TASK_NAME_LEN                 16

/* Tick计数使用32位 */
#define configUSE_16_BIT_TICKS                  0

/* 空闲任务可让出CPU */
#define configIDLE_SHOULD_YIELD                 1

/*-----------------------------------------------------------
 * 同步与通信能力
 *----------------------------------------------------------*/

/* 支持互斥量 */
#define configUSE_MUTEXES                       1

/* 支持递归互斥量，可先打开，后续有需要再用 */
#define configUSE_RECURSIVE_MUTEXES             1

/* 支持计数信号量 */
#define configUSE_COUNTING_SEMAPHORES           1

/* 支持队列注册表，阶段一非必须，可关闭 */
#define configQUEUE_REGISTRY_SIZE               0

/* 任务通知功能 */
#define configUSE_TASK_NOTIFICATIONS            1

/*-----------------------------------------------------------
 * 内存失败与调试
 *----------------------------------------------------------*/

/* 打开 malloc 失败 Hook */
#define configUSE_MALLOC_FAILED_HOOK            1

/* 打开栈溢出检测，2 比 1 更严格 */
#define configCHECK_FOR_STACK_OVERFLOW          2

/* 断言，调试阶段强烈建议启用 */
#define configASSERT( x )                       \
    if( ( x ) == 0 )                            \
    {                                           \
        taskDISABLE_INTERRUPTS();               \
        for( ;; );                              \
    }

/*-----------------------------------------------------------
 * 软件定时器
 * 阶段一暂不使用，可先关闭
 *----------------------------------------------------------*/

#define configUSE_TIMERS                        0

/*-----------------------------------------------------------
 * 与中断优先级相关的配置
 * 后续阶段二会详细接管 SysTick/PendSV/SVC，
 * 阶段一先给出 Cortex-M3 合理模板。
 *----------------------------------------------------------*/

/* STM32F1 通常实现 4 位优先级 */
#define configPRIO_BITS                         4

/* 最低内核中断优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* 允许调用 FreeRTOS FromISR API 的最高中断优先级 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY         \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY    \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/*-----------------------------------------------------------
 * Cortex-M 中断函数名称映射
 * 注意：真正接管 SysTick/PendSV/SVC 在阶段二处理。
 * 这里先预留宏。
 *----------------------------------------------------------*/

#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

/*-----------------------------------------------------------
 * 需要包含的 API 函数
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1

#endif /* FREERTOS_CONFIG_H */
