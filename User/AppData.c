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

/* 静态平衡角补偿
 * 用于抵消小车无速度指令时的固定方向慢滑
 */
float BalanceAngleTrim = 2.0f;

/* GY 启动前静止零偏标定状态 */
volatile uint8_t GyroCalibRequest = 0;
volatile uint8_t GyroCalibBusy = 0;
volatile uint8_t GyroCalibDone = 0;

/* 按键与运行状态 */
uint8_t KeyNum;
volatile uint8_t RunFlag;

/* =========================================================
 * 系统状态机
 * 初始状态为 STOP，等待按键启动标定。
 * ========================================================= */
volatile SystemState_t SystemState = SYS_STOP;
volatile FaultCode_t FaultCode = FAULT_NONE;

/* =========================================================
 * 控制周期实时性诊断
 * ControlOverrunCount：累计出现控制任务积压的次数
 * ControlMaxNotifyCount：记录最大一次积压了多少个 TIM1 通知
 * ========================================================= */
volatile uint32_t ControlOverrunCount = 0;
volatile uint32_t ControlMaxNotifyCount = 0;

/* =========================================================
 * 目标命令变量
 * joystick 输入先写入这里，避免直接突变 PID.Target。
 * ========================================================= */
float SpeedTargetCmd = 0.0f;
float TurnTargetCmd = 0.0f;

/* =========================================================
 * OLED 页码变量
 * K2 按键切换 0 / 1 两页。
 * ========================================================= */
volatile uint8_t DisplayPage = 0;

/* 电机输出相关 */
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

/* 编码器速度反馈 */
float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

/* 角度环 PID */
PID_t AnglePID = {
    .Kp = 1.55,
    //.Kp = 2.4,
    .Ki = 0.1,
    //.Ki = 0.25,
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
