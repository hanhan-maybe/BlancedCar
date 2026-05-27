#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSTasks.h"
#include "AppData.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>


int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    OLED_Init();
    MPU6050_Init();
	
    BlueSerial_Init();
    LED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();

    /*
     * 当前阶段仍保留 Timer_Init。
     * 原 TIM1 中断里的控制逻辑暂时不动。
     */
    Timer_Init();

    /*
     * 创建 FreeRTOS 应用任务：
     * - 按键/LED任务
     * - OLED显示任务
     * - 蓝牙解析任务
     */
    FreeRTOSTasks_Create();

    /*
     * 启动调度器
     */
    vTaskStartScheduler();

    /*
     * 正常情况下不会执行到这里。
     * 若进入，通常表示 FreeRTOS 堆不足，调度器启动失败。
     */
    while (1)
    {
    }
}


/*关于OLED显示屏在修改TIM基础为10ms ，并加入控制任务后不显示这一问题的测试TIM1程序*/
/*
*关掉 Timer_Init 后 OLED 恢复继续查：
*可能在两类：
*TIM1 ISR 中调用 FromISR API 出问题
*ControlTask 被唤醒后运行出问题
*/
/** 
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        
    }
}
*/


void TIM1_UP_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        /*
         * TIM1 中断只做一件事：
         * 通知最高优先级 ControlTask 执行一次控制周期
		 * */
        if (ControlTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(
                ControlTaskHandle,
                &xHigherPriorityTaskWoken
            );
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



/*串口和蓝牙串口测试*/
// int main(void)
// {
// 	OLED_Init();
// 	Serial_Init();
// 	BlueSerial_Init();
	
// 	Serial_SendString("Hello");
// 	Serial_Printf("World");
	
// 	BlueSerial_SendString("hello");
// 	BlueSerial_Printf("world");
	
// 	while (1)
// 	{
// 		if (Serial_GetRxFlag() == 1)
// 		{
// 			uint8_t RxData = Serial_GetRxData();
			
// 			OLED_Printf(0, 0, OLED_8X16, "RxData:%02X", RxData);
// 			OLED_Update();
// 		}
		
// 		if (BlueSerial_RxFlag == 1)
// 		{
// 			OLED_Printf(0, 16, OLED_8X16, "%s", BlueSerial_RxPacket);
// 			OLED_Update();
			
// 			BlueSerial_RxFlag = 0;
// 		}
// 	}
// }
