#ifndef __FREERTOS_TASKS_H
#define __FREERTOS_TASKS_H

#include "FreeRTOS.h"
#include "task.h"


/* 控制任务句柄：TIM1中断需要通过它发送通知 */
extern TaskHandle_t ControlTaskHandle;

/* 创建应用任务 */
void FreeRTOSTasks_Create(void);

#endif
