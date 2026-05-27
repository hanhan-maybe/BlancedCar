#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "stm32f10x.h"
#include "PID.h"

/* =========================================================
 * 系统状态机定义
 * ========================================================= */

/* 系统运行状态 */
typedef enum
{
    SYS_STOP = 0,      // 停止状态，电机关闭，等待启动
    SYS_CALIB,         // 启动前陀螺仪零偏标定状态
    SYS_RUN,           // 正常平衡运行状态
    SYS_FAULT          // 故障保护状态，禁止电机输出
} SystemState_t;

/* 故障码定义 */
typedef enum
{
    FAULT_NONE = 0,        // 无故障
    FAULT_ANGLE_OVER,      // 倾角越界故障
    FAULT_CONTROL_OVERRUN, // 控制任务周期超时故障
    FAULT_MPU_ERROR        // MPU6050 数据异常，当前预留
} FaultCode_t;


/* MPU6050 原始数据 */
extern int16_t AX, AY, AZ, GX, GY, GZ;

/* 定时器调试变量 */
extern uint8_t TimerErrorFlag;
extern uint16_t TimerCount;

/* 姿态解算变量 */
extern float AngleAcc;
extern float AngleGyro;
extern float Angle;

/* GY 零偏自适应补偿相关 */
extern float GyroYBias;       // GY 零偏估计值
extern float GYRaw;           // GY 原始值转 float
extern float GYCorrected;     // GY 零偏补偿后结果

/* GY 启动前静止零偏标定状态 */
extern volatile uint8_t GyroCalibRequest;   // 1: 请求开始标定
extern volatile uint8_t GyroCalibBusy;      // 1: 正在标定
extern volatile uint8_t GyroCalibDone;      // 1: 本次标定已完成

/* 按键与运行状态 */
extern uint8_t KeyNum;
extern volatile uint8_t RunFlag;

/* 电机输出相关 */
extern int16_t LeftPWM, RightPWM;
extern int16_t AvePWM, DifPWM;

/* 编码器速度反馈 */
extern float LeftSpeed, RightSpeed;
extern float AveSpeed, DifSpeed;

/* 静态平衡角补偿 */
extern float BalanceAngleTrim;

/* 三环 PID 控制器 */
extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;

/* =========================================================
 * 系统状态与故障码
 * ========================================================= */
extern volatile SystemState_t SystemState;
extern volatile FaultCode_t FaultCode;

/* =========================================================
 * FreeRTOS 控制周期诊断变量
 * ========================================================= */
extern volatile uint32_t ControlOverrunCount;     // 控制任务超时累计次数
extern volatile uint32_t ControlMaxNotifyCount;   // 单次最大通知积压数

/* =========================================================
 * 速度/转向命令目标
 * 蓝牙 joystick 只修改这两个变量；
 * ControlTask 中再通过 Ramp_Update 平滑更新 PID.Target。
 * ========================================================= */
extern float SpeedTargetCmd;
extern float TurnTargetCmd;

/* =========================================================
 * OLED 显示页
 * 0：运行状态页
 * 1：系统诊断页
 * ========================================================= */
extern volatile uint8_t DisplayPage;

#endif
