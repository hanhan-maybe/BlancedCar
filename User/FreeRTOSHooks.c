#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  FreeRTOS 动态内存申请失败回调
  * @note   当 pvPortMalloc() 分配失败时触发
  */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /* 可后续添加LED报警或串口调试 */
    }
}

/**
  * @brief  FreeRTOS 任务栈溢出回调
  * @param  xTask       发生栈溢出的任务句柄
  * @param  pcTaskName  发生栈溢出的任务名
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();

    while (1)
    {
        /* 可后续添加LED报警或串口调试 */
    }
}
