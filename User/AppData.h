#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "stm32f10x.h"
#include "PID.h"

/* MPU6050 原始数据 */
extern int16_t AX, AY, AZ, GX, GY, GZ;

/* 定时器调试变量 */
extern uint8_t TimerErrorFlag;
extern uint16_t TimerCount;

/* 姿态解算变量 */
extern float AngleAcc;
extern float AngleGyro;
extern float Angle;

/* 按键与运行状态 */
extern uint8_t KeyNum;
extern volatile uint8_t RunFlag;

/* 电机输出相关 */
extern int16_t LeftPWM, RightPWM;
extern int16_t AvePWM, DifPWM;

/* 编码器速度反馈 */
extern float LeftSpeed, RightSpeed;
extern float AveSpeed, DifSpeed;

/* 三环 PID 控制器 */
extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;

#endif
