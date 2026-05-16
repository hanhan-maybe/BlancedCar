#include "FreeRTOSTasks.h"

#include "AppData.h"
#include "LED.h"
#include "Key.h"
#include "OLED.h"
#include "BlueSerial.h"
#include "PID.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

/*-----------------------------------------------------------
 * 任务句柄
 *ControlTaskHandle 不能加 static
 *因为 TIM1_UP_IRQHandler() 要访问它
 *----------------------------------------------------------*/
TaskHandle_t ControlTaskHandle = NULL;
static TaskHandle_t KeyLedTaskHandle = NULL;
static TaskHandle_t DisplayTaskHandle = NULL;
static TaskHandle_t CommTaskHandle = NULL;

/*-----------------------------------------------------------
 * 任务函数声明
 *----------------------------------------------------------*/
static void AppTask_Control(void *pvParameters);
static void AppTask_KeyLed(void *pvParameters);
static void AppTask_Display(void *pvParameters);
static void AppTask_Comm(void *pvParameters);

/*-----------------------------------------------------------
 * 创建任务
 *----------------------------------------------------------*/
void FreeRTOSTasks_Create(void)
{
    BaseType_t ret;

    ret = xTaskCreate(
        AppTask_Control,
        "Control",
        512,
        NULL,
        5,
        &ControlTaskHandle
    );

   configASSERT(ret == pdPASS);

   ret = xTaskCreate(
        AppTask_KeyLed,      // 1. 任务函数名 (这个任务具体要去执行哪个函数)
        "KeyLed",            // 2. 任务的文本名字 (纯粹为了调试好看，没实际作用)
        256,                 // 3. 分配给这个任务的堆栈大小 (256个字，即 1024 字节)
        NULL,                // 4. 传递给任务函数的参数 (这里没传参数，写 NULL)
        3,                   // 5. 任务优先级！(数字越大，优先级越高)
        &KeyLedTaskHandle    // 6. 任务句柄 (相当于这个任务的“身份证号”，后续操作它要用到)
    );

    configASSERT(ret == pdPASS);

    ret = xTaskCreate(
        AppTask_Display,
        "Display",
        512,
        NULL,
        1,
        &DisplayTaskHandle
    );

    configASSERT(ret == pdPASS);

    ret = xTaskCreate(
        AppTask_Comm,
        "Comm",
        512,
        NULL,
        2,
        &CommTaskHandle
    );

    configASSERT(ret == pdPASS);
}


static void AppTask_Control(void *pvParameters)
{
    (void)pvParameters;

    uint8_t SpeedTurnCount = 0;

    for (;;)
    {
        /*
         * 等待 TIM1 中断通知。
         * 返回值表示这段时间累计收到了多少次通知。
         */
        uint32_t NotifyCount = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 若 NotifyCount > 1，说明控制任务上一次可能没有在10ms内及时处理完，
         * 出现了控制节拍积压。
         */
        if (NotifyCount > 1)
        {
            TimerErrorFlag = 1;
        }

        /* =========================================================
         * 10 ms 控制周期：姿态解算 + 角度环 + 电机输出
         * ========================================================= */

        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

        GY -= 16;

        AngleAcc = -atan2(AX, AZ) / 3.14159f * 180.0f;
        AngleAcc += 2.5f;

        AngleGyro = Angle + GY / 32768.0f * 2000.0f * 0.01f;

        {
            float Alpha = 0.01f;
            Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;
        }

        if (Angle > 50.0f || Angle < -50.0f)
        {
            RunFlag = 0;
        }

        if (RunFlag)
        {
            AnglePID.Actual = Angle;
            PID_Update(&AnglePID);

            AvePWM = -AnglePID.Out;

            LeftPWM = AvePWM + DifPWM / 2;
            RightPWM = AvePWM - DifPWM / 2;

            if (LeftPWM > 100)
            {
                LeftPWM = 100;
            }
            else if (LeftPWM < -100)
            {
                LeftPWM = -100;
            }

            if (RightPWM > 100)
            {
                RightPWM = 100;
            }
            else if (RightPWM < -100)
            {
                RightPWM = -100;
            }

            Motor_SetPWM(1, LeftPWM);
            Motor_SetPWM(2, RightPWM);
        }
        else
        {
            Motor_SetPWM(1, 0);
            Motor_SetPWM(2, 0);
        }

        /* =========================================================
         * 50 ms 控制周期：速度环 + 转向环
         * 10 ms ControlTask 执行5次 = 50 ms
         * ========================================================= */

        SpeedTurnCount++;

        if (SpeedTurnCount >= 5)
        {
            SpeedTurnCount = 0;

            LeftSpeed = Encoder_Get(1) / 44.0f / 0.05f / 9.27666f;
            RightSpeed = Encoder_Get(2) / 44.0f / 0.05f / 9.27666f;

            AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;
            DifSpeed = LeftSpeed - RightSpeed;

            if (RunFlag)
            {
                SpeedPID.Actual = AveSpeed;
                PID_Update(&SpeedPID);
                AnglePID.Target = SpeedPID.Out;

                TurnPID.Actual = DifSpeed;
                PID_Update(&TurnPID);
                DifPWM = TurnPID.Out;
            }
        }
    }
}
/*-----------------------------------------------------------
 * 按键任务
 *
 *
 *----------------------------------------------------------*/
static void AppTask_KeyLed(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        /*
         * 保持原程序按键时序：
         * 原来 Key_Tick() 是每1ms由TIM1中断调用一次。
         */
        Key_Tick();

        if (RunFlag)
        {
            LED_ON();
        }
        else
        {
            LED_OFF();
        }

        KeyNum = Key_GetNum();

        if (KeyNum == 1)
        {
            if (RunFlag == 0)
            {
                PID_Init(&AnglePID);
                PID_Init(&SpeedPID);
                PID_Init(&TurnPID);
                RunFlag = 1;
            }
            else
            {
                RunFlag = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*-----------------------------------------------------------
 * OLED显示任务
 *
 * 原 main while(1) 中的 OLED 显示逻辑迁移到这里。
 * OLED刷新不需要很快，建议100ms刷新一次。
 *----------------------------------------------------------*/
static void AppTask_Display(void *pvParameters)
{
    (void)pvParameters;
    

    for (;;)
    {
        OLED_Clear();
        /*观察当前剩余堆空间 和 启动以来最少剩余堆空间*/
        //size_t FreeHeap = xPortGetFreeHeapSize();
        //size_t MinEverHeap = xPortGetMinimumEverFreeHeapSize();

        OLED_Printf(0, 0, OLED_6X8, "  Angle");
        OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
        OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
        OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
        OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
        OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);
        OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);
        OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);

        /*观察当前剩余堆空间 和 启动以来最少剩余堆空间*/
        /*OLED_Printf(56, 56, OLED_6X8, "H:%04u M:%04u",
            (unsigned int)FreeHeap,
            (unsigned int)MinEverHeap);
            */

        OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.OutOffset);

        OLED_Printf(50, 0, OLED_6X8, "Speed");
        OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
        OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
        OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
        OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);
        OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);
        OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);

        OLED_Printf(88, 0, OLED_6X8, "Turn");
        OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
        OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
        OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
        OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
        OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);
        OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);

        OLED_Update();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*-----------------------------------------------------------
 * 蓝牙通信解析任务
 *
 * 原 main while(1) 中 BlueSerial_RxFlag 的解析逻辑迁移到这里。
 * 目前仍保留“轮询标志位”的方式，下一步可再升级为任务通知。
 *----------------------------------------------------------*/
static void AppTask_Comm(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (BlueSerial_RxFlag == 1)
        {
            char *Tag = strtok(BlueSerial_RxPacket, ",");

            if (Tag == NULL)
            {
                /* 数据异常，直接丢弃 */
            }
            else if (strcmp(Tag, "key") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Action = strtok(NULL, ",");

                (void)Name;
                (void)Action;

                /* 当前项目仍保持预留，不做动作 */
            }
            else if (strcmp(Tag, "slider") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");

                if (Name != NULL && Value != NULL)
                {
                    if (strcmp(Name, "AngleKp") == 0)
                    {
                        AnglePID.Kp = atof(Value);
                    }
                    else if (strcmp(Name, "AngleKi") == 0)
                    {
                        AnglePID.Ki = atof(Value);
                    }
                    else if (strcmp(Name, "AngleKd") == 0)
                    {
                        AnglePID.Kd = atof(Value);
                    }
                    else if (strcmp(Name, "SpeedKp") == 0)
                    {
                        SpeedPID.Kp = atof(Value);
                    }
                    else if (strcmp(Name, "SpeedKi") == 0)
                    {
                        SpeedPID.Ki = atof(Value);
                    }
                    else if (strcmp(Name, "SpeedKd") == 0)
                    {
                        SpeedPID.Kd = atof(Value);
                    }
                    else if (strcmp(Name, "TurnKp") == 0)
                    {
                        TurnPID.Kp = atof(Value);
                    }
                    else if (strcmp(Name, "TurnKi") == 0)
                    {
                        TurnPID.Ki = atof(Value);
                    }
                    else if (strcmp(Name, "TurnKd") == 0)
                    {
                        TurnPID.Kd = atof(Value);
                    }
                    else if (strcmp(Name, "Offset") == 0)
                    {
                        AnglePID.OutOffset = atof(Value);
                    }
                }
            }
            else if (strcmp(Tag, "joystick") == 0)
            {
                char *LHStr = strtok(NULL, ",");
                char *LVStr = strtok(NULL, ",");
                char *RHStr = strtok(NULL, ",");
                char *RVStr = strtok(NULL, ",");

                if (LHStr != NULL &&
                    LVStr != NULL &&
                    RHStr != NULL &&
                    RVStr != NULL)
                {
                    int8_t LH = atoi(LHStr);
                    int8_t LV = atoi(LVStr);
                    int8_t RH = atoi(RHStr);
                    int8_t RV = atoi(RVStr);

                    (void)LH;
                    (void)RV;

                    SpeedPID.Target = LV / 25.0f;
                    TurnPID.Target = RH / 25.0f;
                }
            }

            BlueSerial_RxFlag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
