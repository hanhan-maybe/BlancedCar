#include "AppData.h"

/* MPU6050 原始数据 */
int16_t AX, AY, AZ, GX, GY, GZ;

/* 定时器调试变量 */
uint8_t TimerErrorFlag;
uint16_t TimerCount;

/* 姿态解算变量 */
float AngleAcc;
float AngleGyro;
float Angle;

/* GY 零偏自适应补偿相关 */
float GyroYBias;
float GYRaw;
float GYCorrected;

/* GY 启动前静止零偏标定状态 */
volatile uint8_t GyroCalibRequest = 0;
volatile uint8_t GyroCalibBusy = 0;
volatile uint8_t GyroCalibDone = 0;

/* 按键与运行状态 */
uint8_t KeyNum;
volatile uint8_t RunFlag;

/* 电机输出相关 */
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

/* 编码器速度反馈 */
float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

/* 角度环 PID */
PID_t AnglePID = {
    .Kp = 2.4,
    .Ki = 0.25,
    .Kd = 6,

    .OutMax = 100,
    .OutMin = -100,

    .OutOffset = 3,

    .ErrorIntMax = 600,
    .ErrorIntMin = -600,
};

/* 速度环 PID */
PID_t SpeedPID = {
    .Kp = 2.2,
    .Ki = 0.1,
    .Kd = 0,

    .OutMax = 20,
    .OutMin = -20,

    .ErrorIntMax = 150,
    .ErrorIntMin = -150,
};

/* 转向环 PID */
PID_t TurnPID = {
    .Kp = 5.5,
    .Ki = 0.07,
    .Kd = 0,

    .OutMax = 50,
    .OutMin = -50,

    .ErrorIntMax = 20,
    .ErrorIntMin = -20,
};
