- [STM32-Balanced-Car](#stm32-balanced-car)
  - [项目简介](#项目简介)
  - [项目整体框架](#项目整体框架)
  - [工程目录结构](#工程目录结构)
  - [系统运行流程](#系统运行流程)
    - [主循环任务](#主循环任务)
    - [TIM1定时中断任务](#tim1定时中断任务)
  - [姿态检测模块](#姿态检测模块)
    - [MPU6050模块](#mpu6050模块)
    - [软件I2C通信模块](#软件i2c通信模块)
    - [姿态角计算](#姿态角计算)
  - [平衡控制模块](#平衡控制模块)
    - [三级PID控制结构](#三级pid控制结构)
    - [角度环](#角度环)
    - [速度环](#速度环)
    - [转向环](#转向环)
    - [PID通用算法模块](#pid通用算法模块)
    - [控制量融合与电机输出](#控制量融合与电机输出)
  - [电机驱动模块](#电机驱动模块)
    - [PWM输出模块](#pwm输出模块)
    - [电机方向控制模块](#电机方向控制模块)
  - [编码器测速模块](#编码器测速模块)
  - [人机交互模块](#人机交互模块)
    - [OLED显示模块](#oled显示模块)
    - [按键模块](#按键模块)
    - [LED状态指示模块](#led状态指示模块)
  - [串口通信模块](#串口通信模块)
    - [蓝牙串口模块](#蓝牙串口模块)
    - [普通串口模块](#普通串口模块)
    - [蓝牙数据协议](#蓝牙数据协议)
  - [系统定时与任务调度模块](#系统定时与任务调度模块)
  - [各模块调用关系](#各模块调用关系)
  - [项目控制流程总结](#项目控制流程总结)

# STM32-Balanced-Car

本项目是一个基于 **STM32F103** 的两轮自平衡小车控制系统。  
工程通过 **MPU6050 姿态采集、编码器速度反馈、三级 PID 闭环控制、电机 PWM 驱动、蓝牙参数调节与 OLED 实时显示**，实现小车的：

- 原地自平衡
- 前进 / 后退速度控制
- 左右转向控制
- 手机蓝牙远程调参
- OLED 实时显示控制状态
- 倾倒保护与启停控制

本说明文档沿用示例 README 的章节组织方式，并结合当前 ZIP 工程的实际代码结构整理。:contentReference[oaicite:0]{index=0}

---

## 项目简介

该项目的核心思想是：

1. **通过 MPU6050 获取车体倾角与角速度**
2. **通过编码器获得左右轮转速**
3. **使用角度环、速度环、转向环三个 PID 控制器协同工作**
4. **将控制结果转化为左右电机 PWM**
5. **利用蓝牙串口动态调整 PID 参数与运动目标**
6. **使用 OLED 显示系统运行状态和参数**

当前工程采用的是 **裸机中断式控制架构**：

- `while(1)` 主循环：负责按键、OLED刷新、蓝牙指令解析
- `TIM1_UP_IRQHandler()`：负责周期性传感器读取、姿态解算、PID运算和电机控制

---

## 项目整体框架

1. 系统进入 `main`
2. 初始化 OLED、MPU6050、蓝牙串口、LED、按键、电机、编码器、普通串口
3. 初始化 TIM1 定时器中断
4. 进入主循环
5. 主循环完成：
   - 启停控制
   - OLED 状态显示
   - 蓝牙指令解析
   - PID 参数动态更新
6. TIM1 定时中断完成：
   - 按键扫描节拍
   - 姿态数据读取
   - 互补滤波计算角度
   - 角度环控制
   - 编码器测速
   - 速度环控制
   - 转向环控制
   - 左右电机 PWM 输出

---

## 工程目录结构

```text
BlancedCar
├── Hardware
│   ├── BlueSerial.c / .h     蓝牙串口通信
│   ├── Encoder.c / .h        编码器测速
│   ├── Key.c / .h            按键扫描
│   ├── LED.c / .h            LED状态灯
│   ├── Motor.c / .h          电机方向控制
│   ├── MPU6050.c / .h        MPU6050寄存器驱动
│   ├── MPU6050_Reg.h         MPU6050寄存器宏定义
│   ├── MyI2C.c / .h          软件I2C总线
│   ├── OLED.c / .h           OLED显示驱动
│   ├── OLED_Data.c / .h      OLED字库与图像数据
│   ├── PWM.c / .h            PWM输出
│   └── Serial.c / .h         普通串口通信
│
├── System
│   ├── Delay.c / .h          延时函数
│   └── Timer.c / .h          TIM1基础定时器
│
├── User
│   ├── main.c                主程序与控制中断
│   ├── PID.c / .h            PID通用算法
│   ├── stm32f10x_conf.h
│   └── stm32f10x_it.c / .h
│
├── Library                    STM32标准外设库
├── Start                      启动文件与系统配置
└── Project.uvprojx            Keil工程文件
````

---

## 系统运行流程

### 主循环任务

`main()` 中主要完成非实时任务：

1. 根据 `RunFlag` 控制 LED 状态
2. 读取按键事件
3. 根据按键切换小车运行 / 停止状态
4. 刷新 OLED 显示界面
5. 解析蓝牙接收数据
6. 根据蓝牙指令：

   * 调整 PID 参数
   * 调整角度环补偿量 `OutOffset`
   * 更新速度目标与转向目标

主循环承担的是 **交互层逻辑**，不负责高频闭环控制。

---

### TIM1定时中断任务

`TIM1_UP_IRQHandler()` 是整个系统的实时控制核心。

其内部按不同节拍执行多类任务：

#### 1. 每次进入中断

* 清除 TIM1 更新中断标志
* 执行 `Key_Tick()` 按键扫描节拍

#### 2. 每 10 个中断周期执行一次

完成高频姿态控制：

1. 读取 MPU6050 的加速度与角速度
2. 对陀螺仪 Y 轴进行零偏补偿
3. 计算加速度角度
4. 使用互补滤波得到最终车体倾角
5. 检测倾倒保护
6. 执行角度环 PID
7. 计算左右轮基础 PWM
8. 输出电机控制量

#### 3. 每 50 个中断周期执行一次

完成较慢的速度与转向控制：

1. 读取左右编码器计数
2. 换算左右轮速度
3. 计算平均速度与差速
4. 执行速度环 PID
5. 将速度环输出作为角度环目标角
6. 执行转向环 PID
7. 生成差速补偿量 `DifPWM`

---

## 姿态检测模块

### MPU6050模块

文件：

```text
Hardware/MPU6050.c
Hardware/MPU6050.h
Hardware/MPU6050_Reg.h
```

该模块负责：

* 配置 MPU6050 工作模式
* 读取加速度计与陀螺仪原始数据
* 对外提供统一的数据获取接口

初始化函数：

```c
void MPU6050_Init(void);
```

主要配置内容：

```c
MPU6050_PWR_MGMT_1 = 0x01;
MPU6050_PWR_MGMT_2 = 0x00;
MPU6050_SMPLRT_DIV = 0x07;
MPU6050_CONFIG = 0x00;
MPU6050_GYRO_CONFIG = 0x18;
MPU6050_ACCEL_CONFIG = 0x18;
```

数据读取函数：

```c
void MPU6050_GetData(
    int16_t *AccX,
    int16_t *AccY,
    int16_t *AccZ,
    int16_t *GyroX,
    int16_t *GyroY,
    int16_t *GyroZ
);
```

程序一次性连续读取 14 字节寄存器数据，再拆分出：

* 加速度：`AX, AY, AZ`
* 角速度：`GX, GY, GZ`

---

### 软件I2C通信模块

文件：

```text
Hardware/MyI2C.c
Hardware/MyI2C.h
```

该模块使用 GPIO 模拟 I2C 时序，为 MPU6050 提供底层通信能力。

引脚：

| 信号  | STM32引脚 |
| --- | ------- |
| SCL | PB10    |
| SDA | PB11    |

核心函数：

```c
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);
```

功能特点：

* 不依赖硬件 I2C 外设
* 可移植性较高
* 适合本项目这种低速传感器读取场景

---

### 姿态角计算

程序中车体角度由两部分信息融合得到：

#### 1. 加速度计角度

```c
AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
AngleAcc += 2.5;
```

含义：

* 使用 `AX` 与 `AZ` 计算当前倾角
* `+2.5` 为安装偏差或机械零点修正

#### 2. 陀螺仪积分角度

```c
GY -= 16;
AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
```

含义：

* `GY -= 16`：陀螺仪零偏补偿
* 将角速度积分成短时间角度变化

#### 3. 互补滤波融合

```c
float Alpha = 0.01;
Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
```

当前融合逻辑为：

* 1% 使用加速度计角度
* 99% 使用陀螺仪积分角度

这样可以兼顾：

* 加速度计长期稳定
* 陀螺仪短期响应快

---

## 平衡控制模块

### 三级PID控制结构

本项目使用三个 PID 控制器：

```c
PID_t AnglePID;
PID_t SpeedPID;
PID_t TurnPID;
```

它们分别对应：

| 控制环 | 控制目标   | 输出作用     |
| --- | ------ | -------- |
| 角度环 | 车体倾角稳定 | 生成平衡 PWM |
| 速度环 | 小车前后速度 | 改变角度环目标角 |
| 转向环 | 左右轮差速  | 生成转向差速补偿 |

整体结构如下：

```text
速度目标
   ↓
速度环 PID
   ↓
目标倾角 AnglePID.Target
   ↓
角度环 PID
   ↓
平均驱动 PWM
   ↓
左右电机输出

转向目标
   ↓
转向环 PID
   ↓
差速补偿 DifPWM
   ↓
叠加到左右电机
```

---

### 角度环

角度环参数：

```c
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
```

输入：

```c
AnglePID.Actual = Angle;
```

目标：

```c
AnglePID.Target
```

在静止平衡时，角度目标通常接近 0；
当速度环工作时，速度环输出会改变角度环目标，使小车主动前倾或后仰实现移动。

角度环输出：

```c
PID_Update(&AnglePID);
AvePWM = -AnglePID.Out;
```

其输出经过取反后作为左右电机共有的基础驱动力。

---

### 速度环

速度环参数：

```c
PID_t SpeedPID = {
    .Kp = 2.2,
    .Ki = 0.1,
    .Kd = 0,

    .OutMax = 20,
    .OutMin = -20,

    .ErrorIntMax = 150,
    .ErrorIntMin = -150,
};
```

速度反馈：

```c
AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
SpeedPID.Actual = AveSpeed;
```

速度环输出：

```c
PID_Update(&SpeedPID);
AnglePID.Target = SpeedPID.Out;
```

这说明速度环并不直接驱动电机，而是通过改变 **目标倾角** 来实现速度调节：

* 想前进：给一个前倾目标角
* 想后退：给一个后倾目标角
* 想停止：目标角逐步回到接近平衡角

---

### 转向环

转向环参数：

```c
PID_t TurnPID = {
    .Kp = 5.5,
    .Ki = 0.07,
    .Kd = 0,

    .OutMax = 50,
    .OutMin = -50,

    .ErrorIntMax = 20,
    .ErrorIntMin = -20,
};
```

实际差速：

```c
DifSpeed = LeftSpeed - RightSpeed;
TurnPID.Actual = DifSpeed;
```

输出：

```c
PID_Update(&TurnPID);
DifPWM = TurnPID.Out;
```

转向环输出作为左右轮 PWM 的差速修正量：

* 左轮加速、右轮减速：向一侧转向
* 左轮减速、右轮加速：向另一侧转向

---

### PID通用算法模块

文件：

```text
User/PID.c
User/PID.h
```

PID 结构体：

```c
typedef struct {
    float Target;
    float Actual;
    float Actual1;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;
    float ErrorInt;

    float ErrorIntMax;
    float ErrorIntMin;

    float OutMax;
    float OutMin;

    float OutOffset;
} PID_t;
```

#### PID初始化

```c
void PID_Init(PID_t *p);
```

用于清空：

* 目标值
* 实际值
* 输出量
* 当前误差
* 历史误差
* 积分累计量

#### PID更新

```c
void PID_Update(PID_t *p);
```

算法表达式为：

```c
Out =
    Kp * Error0
  + Ki * ErrorInt
  - Kd * (Actual - Actual1);
```

程序中微分项采用的是：

```c
-p->Kd * (p->Actual - p->Actual1)
```

即对 **实际值变化率** 求微分，而不是直接对误差差分求微分。
这种写法可以减小设定值突变时的微分冲击。

#### 积分限幅

```c
if (p->ErrorInt > p->ErrorIntMax)
    p->ErrorInt = p->ErrorIntMax;

if (p->ErrorInt < p->ErrorIntMin)
    p->ErrorInt = p->ErrorIntMin;
```

作用：

* 防止积分饱和
* 降低长时间误差积累导致的过冲

#### 输出补偿量

```c
if (p->Out > 0) p->Out += p->OutOffset;
if (p->Out < 0) p->Out -= p->OutOffset;
```

作用：

* 补偿电机死区
* 当 PID 输出较小但电机无法启动时，额外叠加一段基础驱动力

#### 输出限幅

```c
if (p->Out > p->OutMax) p->Out = p->OutMax;
if (p->Out < p->OutMin) p->Out = p->OutMin;
```

作用：

* 限制输出 PWM 范围
* 防止控制量异常过大

---

### 控制量融合与电机输出

角度环给出平均驱动力：

```c
AvePWM = -AnglePID.Out;
```

转向环给出差速驱动力：

```c
DifPWM = TurnPID.Out;
```

左右轮最终 PWM：

```c
LeftPWM  = AvePWM + DifPWM / 2;
RightPWM = AvePWM - DifPWM / 2;
```

含义：

* `AvePWM` 决定小车前进 / 后退和平衡动作
* `DifPWM` 决定左右转向修正

随后执行限幅：

```c
if (LeftPWM > 100) LeftPWM = 100;
if (LeftPWM < -100) LeftPWM = -100;

if (RightPWM > 100) RightPWM = 100;
if (RightPWM < -100) RightPWM = -100;
```

最后控制电机：

```c
Motor_SetPWM(1, LeftPWM);
Motor_SetPWM(2, RightPWM);
```

---

## 电机驱动模块

### PWM输出模块

文件：

```text
Hardware/PWM.c
Hardware/PWM.h
```

使用：

* 定时器：`TIM2`
* 通道：

  * `TIM2_CH1 -> PA0`
  * `TIM2_CH2 -> PA1`

PWM 初始化设置：

```c
TIM_Period = 100 - 1;
TIM_Prescaler = 36 - 1;
```

PWM 占空比通过比较寄存器设定：

```c
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
```

在本项目中，PWM 范围被控制为：

```text
0 ~ 100
```

对应最终驱动强度。

---

### 电机方向控制模块

文件：

```text
Hardware/Motor.c
Hardware/Motor.h
```

方向控制引脚：

| 电机  | 引脚组       |
| --- | --------- |
| 电机1 | PB12、PB13 |
| 电机2 | PB14、PB15 |

控制逻辑：

#### 电机1

```c
PWM >= 0:
PB12 = 1
PB13 = 0

PWM < 0:
PB12 = 0
PB13 = 1
```

#### 电机2

```c
PWM >= 0:
PB14 = 0
PB15 = 1

PWM < 0:
PB14 = 1
PB15 = 0
```

函数接口：

```c
void Motor_SetPWM(uint8_t n, int8_t PWM);
```

其中：

* `n = 1`：左侧或第一个电机
* `n = 2`：右侧或第二个电机
* `PWM > 0`：正转
* `PWM < 0`：反转
* `PWM = 0`：停止

---

## 编码器测速模块

文件：

```text
Hardware/Encoder.c
Hardware/Encoder.h
```

本项目使用两个定时器的编码器模式：

| 编码器  | 定时器  | 引脚      |
| ---- | ---- | ------- |
| 编码器1 | TIM3 | PA6、PA7 |
| 编码器2 | TIM4 | PB6、PB7 |

测速函数：

```c
int16_t Encoder_Get(uint8_t n);
```

每次读取：

1. 获取当前计数值
2. 清零定时器计数器
3. 返回这段时间内的脉冲累计值

主程序中每个速度控制周期计算：

```c
LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
```

随后计算：

```c
AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
DifSpeed = LeftSpeed - RightSpeed;
```

其中：

* `AveSpeed`：用于速度环
* `DifSpeed`：用于转向环

---

## 人机交互模块

### OLED显示模块

文件：

```text
Hardware/OLED.c
Hardware/OLED.h
Hardware/OLED_Data.c
Hardware/OLED_Data.h
```

OLED 使用软件 I2C 驱动：

| 信号  | STM32引脚 |
| --- | ------- |
| SCL | PB8     |
| SDA | PB9     |

主界面显示三组数据：

#### 1. Angle 区域

* 角度环 `Kp`
* 角度环 `Ki`
* 角度环 `Kd`
* 目标倾角
* 当前倾角
* 角度环输出
* 陀螺仪 Y 轴值
* 死区补偿 `Offset`

#### 2. Speed 区域

* 速度环 `Kp`
* 速度环 `Ki`
* 速度环 `Kd`
* 速度目标
* 实际平均速度
* 速度环输出

#### 3. Turn 区域

* 转向环 `Kp`
* 转向环 `Ki`
* 转向环 `Kd`
* 转向目标
* 实际差速
* 转向环输出

OLED 相关函数：

```c
OLED_Clear();
OLED_Printf();
OLED_Update();
```

---

### 按键模块

文件：

```text
Hardware/Key.c
Hardware/Key.h
```

按键引脚：

| 按键   | 引脚  |
| ---- | --- |
| Key1 | PB1 |
| Key2 | PB0 |
| Key3 | PA5 |
| Key4 | PA4 |

程序使用：

```c
Key_Tick();
```

实现按键状态周期扫描，并采用：

* **松手触发**
* **简单消抖**

方式输出按键编号。

在主程序中：

```c
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
```

因此 Key1 功能是：

* 启动平衡控制
* 停止平衡控制

---

### LED状态指示模块

文件：

```text
Hardware/LED.c
Hardware/LED.h
```

LED 引脚：

| 功能  | 引脚   |
| --- | ---- |
| 状态灯 | PC13 |

程序逻辑：

```c
if (RunFlag)
{
    LED_ON();
}
else
{
    LED_OFF();
}
```

表示：

* 小车运行中：LED亮
* 小车停止：LED灭

---

## 串口通信模块

### 蓝牙串口模块

文件：

```text
Hardware/BlueSerial.c
Hardware/BlueSerial.h
```

蓝牙串口基于：

* 外设：`USART2`
* 波特率：`9600`

引脚：

| 功能 | 引脚  |
| -- | --- |
| TX | PA2 |
| RX | PA3 |

接收中断：

```c
void USART2_IRQHandler(void);
```

采用帧格式：

```text
[数据内容]
```

当收到完整的：

```text
[
...
]
```

后，将内部字符串存入：

```c
BlueSerial_RxPacket
```

并置位：

```c
BlueSerial_RxFlag = 1;
```

---

### 普通串口模块

文件：

```text
Hardware/Serial.c
Hardware/Serial.h
```

普通串口基于：

* 外设：`USART1`
* 波特率：`9600`

引脚：

| 功能 | 引脚   |
| -- | ---- |
| TX | PA9  |
| RX | PA10 |

用途：

* 调试输出
* `printf` 重定向
* 串口数据测试

其中：

```c
int fputc(int ch, FILE *f)
```

将标准输出重定向到串口1。

当前主程序中普通串口主要处于 **初始化但未参与核心控制** 的状态，更多用于调试与测试。

---

### 蓝牙数据协议

程序对三类蓝牙指令进行解析。

---

#### 1. 滑块参数调节

格式：

```text
[slider,参数名,参数值]
```

示例：

```text
[slider,AngleKp,2.4]
[slider,SpeedKi,0.1]
[slider,TurnKd,0]
[slider,Offset,3]
```

支持参数：

| 参数名     | 作用        |
| ------- | --------- |
| AngleKp | 角度环比例系数   |
| AngleKi | 角度环积分系数   |
| AngleKd | 角度环微分系数   |
| SpeedKp | 速度环比例系数   |
| SpeedKi | 速度环积分系数   |
| SpeedKd | 速度环微分系数   |
| TurnKp  | 转向环比例系数   |
| TurnKi  | 转向环积分系数   |
| TurnKd  | 转向环微分系数   |
| Offset  | 角度环输出死区补偿 |

---

#### 2. 摇杆控制

格式：

```text
[joystick,LH,LV,RH,RV]
```

程序中实际使用：

```c
SpeedPID.Target = LV / 25.0;
TurnPID.Target  = RH / 25.0;
```

含义：

* `LV`：控制前进 / 后退速度
* `RH`：控制左右转向

---

#### 3. 按键类命令

程序已经预留：

```c
else if (strcmp(Tag, "key") == 0)
{
    char *Name = strtok(NULL, ",");
    char *Action = strtok(NULL, ",");
}
```

当前尚未补充具体执行逻辑，可用于后续实现：

* 手机端启停控制
* 模式切换
* 参数保存
* 一键复位

---

## 系统定时与任务调度模块

文件：

```text
System/Timer.c
System/Timer.h
```

定时器：

```text
TIM1
```

主要功能：

* 周期性触发实时控制中断
* 为整个系统提供统一的控制节拍

初始化配置：

```c
TIM_Period = 1000 - 1;
TIM_Prescaler = 72 - 1;
```

在常见 72 MHz 系统时钟配置下，可形成约 1 ms 的基础中断节拍。

在 `TIM1_UP_IRQHandler()` 内部又进一步划分出：

| 节拍      | 任务              |
| ------- | --------------- |
| 1 ms级   | 中断基础节拍、按键扫描驱动   |
| 约10 ms级 | 姿态读取、角度环控制、电机输出 |
| 约50 ms级 | 编码器测速、速度环、转向环   |

---

## 各模块调用关系

```text
main.c
├── OLED_Init / OLED_Printf / OLED_Update
├── MPU6050_Init
│   └── MyI2C_Init
├── BlueSerial_Init
│   └── USART2_IRQHandler
├── Serial_Init
│   └── USART1_IRQHandler
├── LED_Init / LED_ON / LED_OFF
├── Key_Init / Key_GetNum
│   └── Key_Tick
├── Motor_Init
│   └── PWM_Init
├── Encoder_Init
├── Timer_Init
│   └── TIM1_UP_IRQHandler
└── PID_Init / PID_Update
```

实时控制链路如下：

```text
MPU6050_GetData
   ↓
角度计算 + 互补滤波
   ↓
AnglePID
   ↓
AvePWM
   ↓
Motor_SetPWM

Encoder_Get
   ↓
速度计算
   ├── SpeedPID → AnglePID.Target
   └── TurnPID  → DifPWM
```

---

## 项目控制流程总结

本项目本质上是一个典型的 **两轮倒立摆闭环控制系统**。

其工作链条可概括为：

1. **姿态测量**

   * MPU6050 采集加速度与角速度

2. **姿态融合**

   * 使用互补滤波计算当前倾角

3. **平衡控制**

   * 角度环 PID 保持车体直立

4. **速度控制**

   * 编码器反馈车轮转速
   * 速度环 PID 将速度误差转化为目标倾角

5. **转向控制**

   * 左右轮速度差形成转向反馈
   * 转向环 PID 生成差速控制量

6. **电机执行**

   * 平衡输出与转向输出融合
   * 生成左右轮最终 PWM

7. **远程交互**

   * 蓝牙串口实时修改 PID 参数与运动目标

8. **状态可视化**

   * OLED 实时显示参数与反馈量

因此，项目总体控制结构可以抽象为：

```text
传感器采集
   ↓
状态估计
   ↓
三级PID控制
   ↓
左右轮PWM分配
   ↓
电机执行
   ↓
编码器反馈
   ↺
```

```
```
