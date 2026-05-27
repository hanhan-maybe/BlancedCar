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
#include "ParamStore.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

/* =========================================================
 * 控制周期相关宏
 * CONTROL_DT：ControlTask 由 TIM1 每 10 ms 唤醒一次
 * SPEED_TURN_DIV：速度环/转向环每 3 次控制周期执行一次
 * SPEED_TURN_DT：速度环/转向环实际周期 = 30 ms
 * ========================================================= */
#define CONTROL_DT                      0.01f
#define SPEED_TURN_DIV                  3
#define SPEED_TURN_DT                   (CONTROL_DT * SPEED_TURN_DIV)

/* =========================================================
 * 控制平滑性参数
 * SPEED_TARGET_RAMP_STEP：速度目标每次外环最大变化量
 * TURN_TARGET_RAMP_STEP：转向目标每次外环最大变化量
 * PWM_SLOPE_STEP：PWM 每个 10 ms 周期最大变化量
 * SPEED_INTEGRAL_ANGLE_LIMIT：倾角超过该值时清除速度环积分
 * ========================================================= */
#define SPEED_TARGET_RAMP_STEP          0.05f
#define TURN_TARGET_RAMP_STEP           0.05f
#define PWM_SLOPE_STEP                  8
#define SPEED_INTEGRAL_ANGLE_LIMIT      15.0f

/* 连续超时多少次后进入故障态 */
#define CONTROL_OVERRUN_FAULT_LIMIT     5

/* =========================================================
 * PWM 上一周期输出
 * 用于实现 PWM 输出变化率限制。
 * ========================================================= */
static int16_t LeftPWMLast = 0;
static int16_t RightPWMLast = 0;


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


/* =========================================================
 * 目标斜坡函数
 *
 * 作用：
 * 不让 PID 目标值一步跳到 joystick 命令值，
 * 而是每次最多变化 step。
 *
 * 用于：
 * SpeedPID.Target
 * TurnPID.Target
 * ========================================================= */
static float Ramp_Update(float current, float target, float step)
{
    if (current < target)
    {
        current += step;

        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;

        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

/* =========================================================
 * PWM 变化率限制函数
 *
 * 作用：
 * 限制当前 PWM 相比上一周期的最大变化量，
 * 避免 PID 输出突变导致电机冲击。
 * ========================================================= */
static int16_t Limit_Slope(int16_t last, int16_t target, int16_t step)
{
    if (target > last + step)
    {
        return last + step;
    }
    else if (target < last - step)
    {
        return last - step;
    }
    else
    {
        return target;
    }
}

/* =========================================================
 * 统一停机与控制状态复位函数
 *
 * 作用：
 * 1. 关闭运行标志；
 * 2. 关闭左右电机；
 * 3. 清除速度/转向命令目标；
 * 4. 清除 PID 目标、积分、输出；
 * 5. 清除 PWM 输出和上一周期 PWM；
 *
 * 使用场景：
 * 1. K1 手动停止；
 * 2. 倾角越界；
 * 3. 控制周期连续超时；
 * 4. 从 FAULT 状态恢复到 STOP 前。
 * ========================================================= */
static void Control_StopAndReset(void)
{
    RunFlag = 0;

    /* 立即关闭电机输出 */
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);

    /* 清除 joystick 命令目标 */
    SpeedTargetCmd = 0.0f;
    TurnTargetCmd = 0.0f;

    /* 清除 PID 目标 */
    SpeedPID.Target = 0.0f;
    TurnPID.Target = 0.0f;

    /* 清除 PID 积分，避免再次启动时积分残留 */
    SpeedPID.ErrorInt = 0.0f;
    TurnPID.ErrorInt = 0.0f;
    AnglePID.ErrorInt = 0.0f;

    /* 清除 PID 输出 */
    SpeedPID.Out = 0.0f;
    TurnPID.Out = 0.0f;
    AnglePID.Out = 0.0f;

    /* 清除 PWM 输出量 */
    AvePWM = 0;
    DifPWM = 0;
    LeftPWM = 0;
    RightPWM = 0;

    /* 清除 PWM 斜率限制的历史值 */
    LeftPWMLast = 0;
    RightPWMLast = 0;
}

/* =========================================================
 * 统一故障入口函数
 *
 * 作用：
 * 1. 记录故障码；
 * 2. 切换到 SYS_FAULT 状态；
 * 3. 调用 Control_StopAndReset() 停机保护。
 * ========================================================= */
static void System_EnterFault(FaultCode_t code)
{
    FaultCode = code;
    SystemState = SYS_FAULT;

    Control_StopAndReset();
}


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
    
    /* 连续控制周期超时次数
    * 偶发超时只统计，不立刻停机；
    * 连续多次超时才进入 FAULT。
    */
    static uint8_t OverrunConsecutiveCount = 0;

     /* GY 启动前静止零偏标定变量 */
    static int32_t GyroYSum = 0;
    static uint16_t GyroYSampleCount = 0;

    #define GYRO_Y_CALIB_SAMPLE_NUM 200

    for (;;)
    {
        /*
         * 等待 TIM1 中断通知。
         * 返回值表示这段时间累计收到了多少次通知。
         */
        uint32_t NotifyCount = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* =========================================================
        * 控制周期超时统计
        *
        * NotifyCount > 1 表示 ControlTask 没有及时处理上一次 TIM1 通知，
        * 出现了控制周期积压。
        * ========================================================= */
        if (NotifyCount > 1)
        {
            TimerErrorFlag = 1;
            ControlOverrunCount++;

            if (NotifyCount > ControlMaxNotifyCount)
            {
                ControlMaxNotifyCount = NotifyCount;
            }

            if (OverrunConsecutiveCount < 255)
            {
                OverrunConsecutiveCount++;
            }

            /* 连续多次超时才进入故障态，避免偶发抖动导致误停机 */
            if (OverrunConsecutiveCount >= CONTROL_OVERRUN_FAULT_LIMIT)
            {
                System_EnterFault(FAULT_CONTROL_OVERRUN);
            }
        }
        else
        {
            OverrunConsecutiveCount = 0;
        }

        /* =========================================================
         * 10 ms 控制周期：姿态解算 + 角度环 + 电机输出
         * ========================================================= */

        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

        /* GY 原始值转 float，便于后续调试显示 */
        GYRaw = (float)GY;

        /* =========================================================
        * 按键启动前的 GY 静止零偏标定
        * ========================================================= */
        if (GyroCalibRequest == 1 && RunFlag == 0)
        {
            /* 第一次进入标定 */
            if (GyroCalibBusy == 0)
            {
                GyroCalibBusy = 1;
                GyroYSum = 0;
                GyroYSampleCount = 0;
            }

            /* 累加当前周期的 GY 原始值 */
            GyroYSum += GY;
            GyroYSampleCount++;

            /* 采样完成 */
            if (GyroYSampleCount >= GYRO_Y_CALIB_SAMPLE_NUM)
            {
                /* 得到本次启动前的 GY 零偏 */
                GyroYBias =
                    (float)GyroYSum / (float)GYRO_Y_CALIB_SAMPLE_NUM;

                /* 标定状态复位 */
                GyroCalibRequest = 0;
                GyroCalibBusy = 0;
                GyroCalibDone = 1;

                /*
                * 重新初始化 PID，
                * 确保从干净状态进入平衡控制。
                */
                PID_Init(&AnglePID);
                PID_Init(&SpeedPID);
                PID_Init(&TurnPID);

                SpeedPID.Target = 0;
                TurnPID.Target = 0;
                DifPWM = 0;

                /*
                * 此处继续保持你原来的 AngleAcc 固定修正：
                * AngleAcc += 2.5f
                */
                AngleAcc = -atan2((float)AX, (float)AZ) / 3.14159f * 180.0f;
                AngleAcc += 2.5f;

                /*
                * 进入平衡前，将融合角初始值对齐到当前加速度角，
                * 避免旧 Angle 状态残留影响启动瞬间。
                */
                Angle = AngleAcc;
                AngleGyro = AngleAcc;

                /* =========================================================
                * 标定完成后，清除外环目标和差速输出，
                * 避免旧控制量影响启动瞬间。
                * ========================================================= */
                SpeedTargetCmd = 0.0f;
                TurnTargetCmd = 0.0f;

                SpeedPID.Target = 0.0f;
                TurnPID.Target = 0.0f;
                DifPWM = 0;

                /* 清除故障码，并进入 RUN 状态 */
                FaultCode = FAULT_NONE;
                SystemState = SYS_RUN;

                /* 最后再打开运行标志 */
                RunFlag = 1;
            }
        }

        /* =========================================================
        * 正常姿态解算
        * ========================================================= */

        /* 补偿后的陀螺仪 Y 轴原始量 */
        GYCorrected = GYRaw - GyroYBias;

        /* AngleAcc 仍保持原来的固定补偿，不改 */
        AngleAcc = -atan2((float)AX, (float)AZ) / 3.14159f * 180.0f;
        AngleAcc += 2.5f;

        /* 陀螺积分使用补偿后的 GYCorrected */
        AngleGyro = Angle + GYCorrected / 32768.0f * 2000.0f * 0.01f;

        /* 互补滤波 */
        float Alpha = 0.01f;
        Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;

        /* =========================================================
        * 倾角越界保护
        *
        * 小车倾角过大时，说明已经无法正常恢复平衡。
        * 直接进入故障状态，关闭电机输出。
        * ========================================================= */
        if (Angle > 50.0f || Angle < -50.0f)
        {
            System_EnterFault(FAULT_ANGLE_OVER);
        }

        /* =========================================================
        * 角度环 + 电机输出
        *
        * 运行状态下：
        * 1. 角度环根据当前 Angle 计算平均 PWM；
        * 2. 叠加转向环输出 DifPWM；
        * 3. 对最终 PWM 进行幅值限制；
        * 4. 再进行变化率限制；
        * 5. 输出到电机。
        *
        * 停止状态下：
        * 电机输出为 0，同时清除 PWM 历史值。
        * ========================================================= */
        if (RunFlag)
        {
            AnglePID.Actual = Angle;
            PID_Update(&AnglePID);

            AvePWM = -AnglePID.Out;

            LeftPWM = AvePWM + DifPWM / 2;
            RightPWM = AvePWM - DifPWM / 2;

            /* PWM 幅值限制 */
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

            /* PWM 变化率限制，避免电机输出突变 */
            LeftPWM = Limit_Slope(LeftPWMLast, LeftPWM, PWM_SLOPE_STEP);
            RightPWM = Limit_Slope(RightPWMLast, RightPWM, PWM_SLOPE_STEP);

            LeftPWMLast = LeftPWM;
            RightPWMLast = RightPWM;

            Motor_SetPWM(1, LeftPWM);
            Motor_SetPWM(2, RightPWM);
        }
        else
        {
            Motor_SetPWM(1, 0);
            Motor_SetPWM(2, 0);

            /* 停止状态下清除 PWM 历史值，避免下次启动沿用旧值 */
            LeftPWMLast = 0;
            RightPWMLast = 0;
        }

        /* =========================================================
         * 30 ms 控制周期：速度环 + 转向环
         * 10 ms ControlTask 执行3次 = 30 ms
         * ========================================================= */

        SpeedTurnCount++;

        if (SpeedTurnCount >= 3)
        {
            SpeedTurnCount = 0;

            /* =========================================================
            * 编码器速度换算
            *
            * SPEED_TURN_DT = 0.03f，对应速度/转向环 30 ms 周期。
            * 不再使用原来的 0.05f。
            * ========================================================= */
            LeftSpeed = Encoder_Get(1) / 44.0f / SPEED_TURN_DT / 9.27666f;
            RightSpeed = Encoder_Get(2) / 44.0f / SPEED_TURN_DT / 9.27666f;

            AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;
            DifSpeed = LeftSpeed - RightSpeed;

            if (RunFlag)
            {
                /* =====================================================
                * 目标斜坡：
                * joystick 命令先进入 SpeedTargetCmd / TurnTargetCmd，
                * 再由 Ramp_Update 平滑更新 PID 目标。
                * ===================================================== */
                SpeedPID.Target = Ramp_Update(
                    SpeedPID.Target,
                    SpeedTargetCmd,
                    SPEED_TARGET_RAMP_STEP
                );

                TurnPID.Target = Ramp_Update(
                    TurnPID.Target,
                    TurnTargetCmd,
                    TURN_TARGET_RAMP_STEP
                );

                /* =====================================================
                * 速度环积分分离：
                * 当车体倾角过大时，优先恢复姿态，
                * 不让速度环继续积分，避免恢复时过冲。
                * ===================================================== */
                SpeedPID.Actual = AveSpeed;

                if (fabs(Angle) < SPEED_INTEGRAL_ANGLE_LIMIT)
                {
                    PID_Update(&SpeedPID);
                }
                else
                {
                    SpeedPID.ErrorInt = 0.0f;
                    SpeedPID.Out = 0.0f;
                }

                /*
                * 速度环输出作为角度环目标，
                * BalanceAngleTrim 用于补偿机械安装误差和静态慢滑。
                */
                AnglePID.Target = SpeedPID.Out + BalanceAngleTrim;

                /* 转向环根据左右轮速度差输出差速 PWM */
                TurnPID.Actual = DifSpeed;
                PID_Update(&TurnPID);
                DifPWM = TurnPID.Out;
            }
            else
            {
                /* 停止状态下清除外环状态 */
                SpeedPID.Target = 0.0f;
                TurnPID.Target = 0.0f;

                SpeedPID.Out = 0.0f;
                TurnPID.Out = 0.0f;

                SpeedPID.ErrorInt = 0.0f;
                TurnPID.ErrorInt = 0.0f;

                DifPWM = 0;
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
        //Key_Tick() 的“函数调用频率”是 1 ms 一次；
        //Key_Tick() 内部真正进行“按键采样和事件判定”的频率是 20 ms 一次。
        Key_Tick();
        
        KeyNum = Key_GetNum();

        if (KeyNum == 1)
        {
            if (SystemState == SYS_FAULT)
            {
                /* =================================================
                * 故障状态下按 K1：
                * 只清故障并回到 STOP，不直接启动。
                * 这样可以避免故障刚清除就立刻启动电机。
                * ================================================= */
                Control_StopAndReset();
                FaultCode = FAULT_NONE;
                SystemState = SYS_STOP;
            }
            else if (RunFlag == 0)
            {
                /* =================================================
                * 停止状态下按 K1：
                * 不直接进入平衡，先请求 GY 零偏标定。
                * 标定完成后由 ControlTask 设置 RunFlag = 1。
                * ================================================= */
                if (GyroCalibBusy == 0)
                {
                    GyroCalibRequest = 1;
                    GyroCalibDone = 0;

                    FaultCode = FAULT_NONE;
                    SystemState = SYS_CALIB;
                }
            }
            else
            {
                /* =================================================
                * 运行状态下按 K1：
                * 主动停止并清除控制残留。
                * ================================================= */
                Control_StopAndReset();
                FaultCode = FAULT_NONE;
                SystemState = SYS_STOP;
            }
        }
        else if (KeyNum == 2)
        {
            /* =====================================================
            * K2 切换 OLED 页面
            * 只在 Page 0 和 Page 1 之间切换。
            * ===================================================== */
            DisplayPage = 1;
        }
        /* =========================================================
        * LED 状态指示
        * 当前简化设计：只有 SYS_RUN 时点亮。
        * ========================================================= */
        if (SystemState == SYS_RUN)
        {
            LED_ON();
        }
        else
        {
            LED_OFF();
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

    /* PlotCount 用于限制蓝牙 plot 发送频率。
     * DisplayTask 每 100 ms 执行一次；
     * PlotCount >= 5 即 500 ms 发送一次。
     */
    static uint8_t PlotCount = 0;

    for (;;)
    {
        OLED_Clear();

        if (DisplayPage == 0)
        {
            /* =================================================
             * Page 0：运行状态页
             * 不显示 PID 参数，只显示运行所需核心状态。
             * ================================================= */
            /*测试按键*/
            // OLED_Printf(0, 0, OLED_6X8, "K:%d R:%d", KeyNum, RunFlag);
            // OLED_Printf(0, 8, OLED_6X8, "Req:%d Busy:%d",
            //                 GyroCalibRequest, GyroCalibBusy);
            // OLED_Printf(0, 16, OLED_6X8, "Done:%d", GyroCalibDone);

            OLED_Printf(0, 0, OLED_6X8, "STATE:%d F:%d",
                        SystemState, FaultCode);

            OLED_Printf(0, 8, OLED_6X8, "RUN:%d CAL:%d",
                        RunFlag, GyroCalibBusy);

            OLED_Printf(0, 16, OLED_6X8, "ANG:%+05.1f",
                        Angle);

            OLED_Printf(0, 24, OLED_6X8, "ATG:%+05.1f",
                        AnglePID.Target);

            OLED_Printf(0, 32, OLED_6X8, "SPD:%+05.1f",
                        AveSpeed);

            OLED_Printf(0, 40, OLED_6X8, "STG:%+05.1f",
                        SpeedPID.Target);

            OLED_Printf(0, 48, OLED_6X8, "L:%+04d R:%+04d",
                        LeftPWM, RightPWM);

            OLED_Printf(0, 56, OLED_6X8, "GY:%+05.0f",
                        GYCorrected);
        }
        else
        {
            /* =================================================
             * Page 1：系统诊断页
             * 显示 FreeRTOS 资源和控制周期诊断信息。
             * ================================================= */

            size_t FreeHeap = xPortGetFreeHeapSize();
            size_t MinHeap = xPortGetMinimumEverFreeHeapSize();

            UBaseType_t CtrlStack =
                uxTaskGetStackHighWaterMark(ControlTaskHandle);

            UBaseType_t DispStack =
                uxTaskGetStackHighWaterMark(DisplayTaskHandle);

            OLED_Printf(0, 0, OLED_6X8, "DIAG");

            OLED_Printf(0, 8, OLED_6X8, "Heap:%u",
                        (unsigned int)FreeHeap);

            OLED_Printf(0, 16, OLED_6X8, "MinH:%u",
                        (unsigned int)MinHeap);

            OLED_Printf(0, 24, OLED_6X8, "CStk:%u",
                        (unsigned int)CtrlStack);

            OLED_Printf(0, 32, OLED_6X8, "DStk:%u",
                        (unsigned int)DispStack);

            OLED_Printf(0, 40, OLED_6X8, "Ovr:%lu",
                        ControlOverrunCount);

            OLED_Printf(0, 48, OLED_6X8, "MaxN:%lu",
                        ControlMaxNotifyCount);
            
            // OLED_Printf(0, 56, OLED_6X8, "TErr:%d",TimerErrorFlag);
            OLED_Printf(0, 56, OLED_6X8, "P:S%d L%d V%d",ParamSaveStatus,ParamLoadStatus,ParamVerifyStatus);
        }

        OLED_Update();

        /* =====================================================
         * 蓝牙 plot 限频发送
         * 原来每 100 ms 发送一次，现在改为每 500 ms 发送一次，
         * 降低阻塞式串口发送对系统实时性的影响。
         * ===================================================== */
        PlotCount++;

        if (PlotCount >= 5)
        {
            PlotCount = 0;

            BlueSerial_Printf("[plot,%.2f,%.2f,%.2f,%.2f]",
                              AnglePID.Target,
                              SpeedPID.Out,
                              AveSpeed,
                              SpeedPID.Target);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*-----------------------------------------------------------
 * 蓝牙通信解析任务
 *
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

                    /* =========================================================
                    * joystick 命令不再直接写 PID.Target。
                    * 先写入命令目标，再由 ControlTask 做目标斜坡。
                    * ========================================================= */
                    SpeedTargetCmd = LV / 25.0f;
                    TurnTargetCmd = RH / 25.0f;
                }
            }

            /*增加 Flash 参数保存命令*/
            else if (strcmp(Tag, "save") == 0)
            {
                char *Name = strtok(NULL, ",");

                if (Name != NULL && strcmp(Name, "param") == 0)
                {
                    /* 保存当前 PID 参数和平衡角补偿到片内 Flash */
                    ParamStore_Save();
                }
            }
            else if (strcmp(Tag, "load") == 0)
            {
                char *Name = strtok(NULL, ",");

                if (Name != NULL && strcmp(Name, "param") == 0)
                {
                    /* 从片内 Flash 读取已保存参数 */
                    ParamStore_Load();
                }
            }
            else if (strcmp(Tag, "reset") == 0)
            {
                char *Name = strtok(NULL, ",");

                if (Name != NULL && strcmp(Name, "param") == 0)
                {
                    /* 仅恢复 RAM 中的默认参数，不擦除 Flash */
                    ParamStore_ResetToDefault();
                }
            }

            BlueSerial_RxFlag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
