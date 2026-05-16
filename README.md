# FreeRTOSBlancedCar

* [FreeRTOSBlancedCar](#freertosblancedcar)

  * [一、项目简介](#一项目简介)
  * [二、工程整体功能说明](#二工程整体功能说明)

    * [1. 系统总体功能](#1-系统总体功能)
    * [2. 系统软件架构](#2-系统软件架构)
    * [3. 工程运行流程](#3-工程运行流程)
    * [4. 控制系统总体框架](#4-控制系统总体框架)
    * [5. 工程目录结构](#5-工程目录结构)
  * [三、FreeRTOS任务设计说明](#三freertos任务设计说明)

    * [1. 任务总体划分](#1-任务总体划分)
    * [2. ControlTask：实时控制任务](#2-controltask实时控制任务)
    * [3. KeyLedTask：按键与LED任务](#3-keyledtask按键与led任务)
    * [4. DisplayTask：OLED显示任务](#4-displaytaskoled显示任务)
    * [5. CommTask：蓝牙通信解析任务](#5-commtask蓝牙通信解析任务)
    * [6. TIM1中断与任务通知机制](#6-tim1中断与任务通知机制)
  * [四、硬件模块与工程文件对应关系](#四硬件模块与工程文件对应关系)
  * [五、各硬件模块详细说明](#五各硬件模块详细说明)

    * [1. MPU6050姿态检测模块](#1-mpu6050姿态检测模块)
    * [2. 软件I2C通信模块](#2-软件i2c通信模块)
    * [3. 电机驱动与PWM输出模块](#3-电机驱动与pwm输出模块)
    * [4. 编码器测速模块](#4-编码器测速模块)
    * [5. 按键输入模块](#5-按键输入模块)
    * [6. LED运行状态指示模块](#6-led运行状态指示模块)
    * [7. OLED数据显示模块](#7-oled数据显示模块)
    * [8. 普通串口调试模块](#8-普通串口调试模块)
    * [9. 蓝牙串口通信模块](#9-蓝牙串口通信模块)
    * [10. 定时器调度模块](#10-定时器调度模块)
  * [六、控制算法说明](#六控制算法说明)

    * [1. 三环控制结构](#1-三环控制结构)
    * [2. 姿态角估计](#2-姿态角估计)
    * [3. 角度环PID](#3-角度环pid)
    * [4. 速度环PID](#4-速度环pid)
    * [5. 转向环PID](#5-转向环pid)
    * [6. PID通用实现](#6-pid通用实现)
  * [七、FreeRTOS配置说明](#七freertos配置说明)

    * [1. 系统节拍与调度方式](#1-系统节拍与调度方式)
    * [2. 动态堆配置](#2-动态堆配置)
    * [3. 中断优先级配置](#3-中断优先级配置)
    * [4. Hook函数](#4-hook函数)
  * [八、主程序执行流程](#八主程序执行流程)
  * [九、蓝牙通信协议说明](#九蓝牙通信协议说明)
  * [十、关键文件功能速查表](#十关键文件功能速查表)
  * [十一、当前工程特点与阶段性改造结果](#十一当前工程特点与阶段性改造结果)

---

# FreeRTOSBlancedCar

本项目是一个基于 **STM32F103C8T6 + FreeRTOS** 的两轮自平衡小车控制系统。
系统通过 **MPU6050姿态传感器** 获取车体倾角，通过 **编码器** 获取左右轮速度，通过 **串级 PID 控制算法** 计算电机输出，最终实现：

1. 小车直立自平衡；
2. 前进与后退速度控制；
3. 左右转向控制；
4. OLED 实时数据显示；
5. 蓝牙串口在线调参与遥控；
6. 按键启停与 LED 状态指示；
7. 基于 FreeRTOS 的任务化软件架构；
8. 基于 TIM1 中断通知的周期控制任务调度。

与原始裸机版本相比，当前工程的主要变化是：

* 原来由 `while(1)` 和 `TIM1` 中断共同承担的大量逻辑，已拆分为多个 FreeRTOS 任务；
* 实时控制逻辑从中断服务函数中迁移至 `ControlTask`；
* `TIM1` 中断不再直接进行姿态解算和 PID 计算，而是仅用于**周期性唤醒控制任务**；
* OLED 显示、按键处理、蓝牙命令解析分别运行于独立任务中，软件结构更清晰，后续扩展性更好。

---

## 一、项目简介

两轮平衡车本质上属于典型的 **倒立摆闭环控制系统**。
车体只有两个轮子支撑，在重力作用下天然不稳定，因此需要通过高频反馈控制，使电机及时补偿车体倾倒趋势，从而维持动态平衡。

本工程采用的基本控制思路为：

* **MPU6050** 采集加速度与角速度；
* 使用 **互补滤波** 估计车体倾角；
* 使用 **角度环 PID** 保持车体直立；
* 使用 **速度环 PID** 控制前进、后退与停车；
* 使用 **转向环 PID** 控制左右差速；
* 使用 **PWM + 电机方向控制** 输出执行量；
* 使用 **FreeRTOS 多任务机制** 分离实时控制、交互、显示与通信；
* 使用 **TIM1 更新中断 + 任务通知** 实现 10 ms 控制周期驱动。

---

## 二、工程整体功能说明

### 1. 系统总体功能

整个工程可划分为五类功能：

#### 1）姿态检测功能

负责从 MPU6050 读取：

* 三轴加速度：`AX`、`AY`、`AZ`
* 三轴角速度：`GX`、`GY`、`GZ`

并计算：

* 加速度倾角 `AngleAcc`
* 角速度积分角 `AngleGyro`
* 融合后的车体姿态角 `Angle`

---

#### 2）闭环控制功能

根据姿态、速度与转向反馈完成三环控制：

* 角度环：保持平衡；
* 速度环：控制前进后退；
* 转向环：控制左右差速。

---

#### 3）人机交互功能

包括：

* 按键启停；
* LED 显示运行/停止状态；
* OLED 显示控制参数和实时状态；
* 蓝牙在线调参与遥控。

---

#### 4）底层外设驱动功能

包括：

* 软件 I2C；
* USART1 调试串口；
* USART2 蓝牙串口；
* TIM1 周期中断；
* TIM2 PWM 输出；
* TIM3/TIM4 编码器测速；
* GPIO 电机方向控制。

---

#### 5）FreeRTOS 任务调度功能

通过任务划分实现：

* 控制任务高优先级运行；
* 按键与 LED 任务周期执行；
* OLED 低频刷新；
* 蓝牙数据独立解析；
* 中断只承担最小化触发职责。

---

### 2. 系统软件架构

当前工程已由裸机轮询式结构升级为 FreeRTOS 任务化结构：

```text
                    ┌─────────────────────┐
                    │      main()         │
                    │ 系统初始化 + 建立任务 │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ vTaskStartScheduler │
                    └──────────┬──────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        ▼                      ▼                      ▼
┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│ ControlTask  │      │ KeyLedTask   │      │ DisplayTask  │
│ 最高优先级    │      │ 按键+LED     │      │ OLED刷新      │
└──────┬───────┘      └──────────────┘      └──────────────┘
       ▲
       │ 任务通知
       │
┌──────┴───────┐
│ TIM1 IRQ     │
│ 10ms周期触发 │
└──────────────┘

                ┌──────────────┐
                │ CommTask     │
                │ 蓝牙命令解析 │
                └──────────────┘
```

---

### 3. 工程运行流程

系统启动后的总体流程如下：

1. 进入 `main()`；
2. 初始化各外设模块：

   * OLED；
   * MPU6050；
   * 蓝牙串口；
   * LED；
   * 按键；
   * 电机；
   * 编码器；
   * 普通调试串口；
3. 初始化 `TIM1` 定时中断；
4. 创建 FreeRTOS 应用任务；
5. 启动 FreeRTOS 调度器；
6. 系统进入多任务运行状态：

   * `ControlTask` 等待 TIM1 中断通知；
   * `KeyLedTask` 每 1 ms 执行按键扫描与 LED 更新；
   * `DisplayTask` 每 100 ms 刷新 OLED；
   * `CommTask` 每 5 ms 轮询并解析蓝牙数据。

---

### 4. 控制系统总体框架

项目采用典型的 **三环串级控制结构**：

```text
蓝牙遥控输入
   │
   ├── 速度目标 SpeedPID.Target ──> 速度环 PID ──> AnglePID.Target
   │
   └── 转向目标 TurnPID.Target ──> 转向环 PID ──> DifPWM

MPU6050姿态角 Angle ──> 角度环 PID ──> AvePWM

AvePWM ± DifPWM/2
   │
   ├── LeftPWM  ──> 左电机
   └── RightPWM ──> 右电机
```

其中：

* **角度环** 是最内层、最高实时性的控制环；
* **速度环** 用于动态调整角度环目标值；
* **转向环** 用于输出左右轮差速量。

---

### 5. 工程目录结构

```text
BlancedCar
├── Start/                    # 启动文件、系统时钟、Cortex-M3支持
├── Library/                  # STM32F10x标准外设库
├── FreeRTOS/                 # FreeRTOS内核与堆管理
│   ├── include/
│   ├── source/
│   └── portable/
├── System/                   # 系统级功能模块
│   ├── Delay.c/h
│   └── Timer.c/h
├── Hardware/                 # 硬件驱动层
│   ├── MPU6050.c/h
│   ├── MyI2C.c/h
│   ├── Motor.c/h
│   ├── PWM.c/h
│   ├── Encoder.c/h
│   ├── OLED.c/h
│   ├── OLED_Data.c/h
│   ├── Key.c/h
│   ├── LED.c/h
│   ├── Serial.c/h
│   └── BlueSerial.c/h
├── User/                     # 用户应用层
│   ├── main.c
│   ├── AppData.c/h
│   ├── PID.c/h
│   ├── FreeRTOSTasks.c/h
│   ├── FreeRTOSConfig.h
│   ├── FreeRTOSHooks.c
│   ├── stm32f10x_it.c/h
│   └── stm32f10x_conf.h
├── Project.uvprojx           # Keil工程文件
└── README.md
```

---

## 三、FreeRTOS任务设计说明

### 1. 任务总体划分

当前工程创建了 4 个任务：

| 任务名       | 任务函数              | 优先级 | 栈大小 | 主要功能                |
| --------- | ----------------- | --: | --: | ------------------- |
| `Control` | `AppTask_Control` |   5 | 512 | 姿态解算、三环控制、电机输出      |
| `KeyLed`  | `AppTask_KeyLed`  |   3 | 256 | 按键扫描、运行状态切换、LED控制   |
| `Comm`    | `AppTask_Comm`    |   2 | 512 | 蓝牙数据包解析、在线调参、遥控目标更新 |
| `Display` | `AppTask_Display` |   1 | 512 | OLED 实时数据显示         |

任务优先级设计思路：

```text
ControlTask > KeyLedTask > CommTask > DisplayTask
```

其中：

* 控制任务必须具有最高优先级；
* 按键响应重要但非强实时；
* 蓝牙解析属于普通业务逻辑；
* OLED 显示刷新较慢，优先级最低。

---

### 2. ControlTask：实时控制任务

#### 2.1 任务触发方式

`ControlTask` 不使用普通 `vTaskDelay()` 周期运行，而是：

* 阻塞等待任务通知；
* 由 `TIM1_UP_IRQHandler()` 每 10 ms 唤醒一次。

核心代码逻辑：

```c
uint32_t NotifyCount = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
```

如果通知累计次数大于 1：

```c
if (NotifyCount > 1)
{
    TimerErrorFlag = 1;
}
```

说明控制任务可能未能在上一个 10 ms 周期内及时完成，出现节拍积压。

---

#### 2.2 10 ms 控制内容

每收到一次任务通知，执行一次：

1. 读取 MPU6050 数据；
2. 计算姿态角；
3. 执行互补滤波；
4. 判断倾角是否越界；
5. 执行角度环 PID；
6. 计算左右轮 PWM；
7. 输出电机驱动。

---

#### 2.3 50 ms 控制内容

控制任务内部使用 `SpeedTurnCount` 计数：

```c
SpeedTurnCount++;
if (SpeedTurnCount >= 5)
```

由于 `ControlTask` 周期为 10 ms，因此每 5 次即为：

```text
10 ms × 5 = 50 ms
```

此时执行：

1. 编码器速度采样；
2. 平均速度与差速计算；
3. 速度环 PID；
4. 转向环 PID。

---

### 3. KeyLedTask：按键与LED任务

#### 3.1 执行周期

```c
vTaskDelay(pdMS_TO_TICKS(1));
```

即约每 1 ms 执行一次。

---

#### 3.2 任务内容

1. 调用 `Key_Tick()` 实现按键扫描与消抖；
2. 根据 `RunFlag` 控制 LED：

   * `RunFlag = 1`：LED 亮；
   * `RunFlag = 0`：LED 灭；
3. 读取按键编号：

   ```c
   KeyNum = Key_GetNum();
   ```
4. 若检测到 1 号按键：

   * 当前停止 → 初始化三个 PID 并进入运行；
   * 当前运行 → 立即停止。

---

#### 3.3 按键控制逻辑

```text
按键1松手触发
   │
   ├── 小车停止状态：初始化PID，RunFlag = 1
   └── 小车运行状态：RunFlag = 0
```

---

### 4. DisplayTask：OLED显示任务

#### 4.1 执行周期

```c
vTaskDelay(pdMS_TO_TICKS(100));
```

即每 100 ms 刷新一次 OLED。

---

#### 4.2 显示内容

OLED 分三栏显示：

##### 角度环区域

* `AnglePID.Kp`
* `AnglePID.Ki`
* `AnglePID.Kd`
* 角度目标值 `AnglePID.Target`
* 实际角度 `Angle`
* 角度环输出 `AnglePID.Out`
* 陀螺仪 `GY`
* 输出偏置 `OutOffset`

##### 速度环区域

* `SpeedPID.Kp`
* `SpeedPID.Ki`
* `SpeedPID.Kd`
* 速度目标 `SpeedPID.Target`
* 平均速度 `AveSpeed`
* 输出 `SpeedPID.Out`

##### 转向环区域

* `TurnPID.Kp`
* `TurnPID.Ki`
* `TurnPID.Kd`
* 转向目标 `TurnPID.Target`
* 实际差速 `DifSpeed`
* 输出 `TurnPID.Out`

---

#### 4.3 堆空间调试预留

代码中保留了 FreeRTOS 堆空间观察逻辑：

```c
xPortGetFreeHeapSize();
xPortGetMinimumEverFreeHeapSize();
```

当前被注释，可在需要时恢复，用于评估：

* 当前剩余堆；
* 系统启动以来的最小剩余堆。

---

### 5. CommTask：蓝牙通信解析任务

#### 5.1 执行周期

```c
vTaskDelay(pdMS_TO_TICKS(5));
```

即每 5 ms 检查一次是否收到完整蓝牙数据包。

---

#### 5.2 当前解析的命令类型

支持三类标签：

```text
key
slider
joystick
```

---

#### 5.3 `slider`：在线调参

可修改：

* `AngleKp`
* `AngleKi`
* `AngleKd`
* `SpeedKp`
* `SpeedKi`
* `SpeedKd`
* `TurnKp`
* `TurnKi`
* `TurnKd`
* `Offset`

例如：

```text
[slider,AngleKp,2.4]
[slider,SpeedKi,0.10]
[slider,Offset,3]
```

---

#### 5.4 `joystick`：遥控目标更新

数据格式：

```text
[joystick,LH,LV,RH,RV]
```

其中当前工程实际使用：

* `LV` → 速度目标；
* `RH` → 转向目标。

映射关系：

```c
SpeedPID.Target = LV / 25.0f;
TurnPID.Target = RH / 25.0f;
```

---

#### 5.5 `key`：预留接口

代码中已解析 `key` 命令，但目前未执行具体动作，属于后续扩展接口。

---

### 6. TIM1中断与任务通知机制

#### 6.1 TIM1功能定位

当前 `TIM1` 的职责已经被简化为：

> **每 10 ms 产生一次更新中断，并向 ControlTask 发送任务通知。**

---

#### 6.2 中断服务函数

```c
void TIM1_UP_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

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
```

---

#### 6.3 改造意义

原来中断中直接执行姿态解算和 PID 运算，存在：

* 中断执行时间长；
* 后续扩展困难；
* 与 FreeRTOS 调度冲突风险更高。

当前改造后：

* 中断函数非常短；
* 实际控制逻辑在线程上下文执行；
* 与 FreeRTOS 的系统设计更匹配；
* 便于后续加入队列、事件组或控制状态机。

---

## 四、硬件模块与工程文件对应关系

| 模块            | 主要作用              | 对应文件                                |
| ------------- | ----------------- | ----------------------------------- |
| MPU6050 姿态传感器 | 获取加速度、角速度         | `Hardware/MPU6050.c/h`              |
| 软件 I2C        | 与 MPU6050 通信      | `Hardware/MyI2C.c/h`                |
| 电机方向控制        | 控制左右电机正反转         | `Hardware/Motor.c/h`                |
| PWM 调速        | 输出电机占空比           | `Hardware/PWM.c/h`                  |
| 编码器测速         | 获取左右轮转速           | `Hardware/Encoder.c/h`              |
| 按键输入          | 启停控制与预留按键         | `Hardware/Key.c/h`                  |
| LED 指示        | 显示运行状态            | `Hardware/LED.c/h`                  |
| OLED 显示       | 显示控制参数与状态         | `Hardware/OLED.c/h`、`OLED_Data.c/h` |
| 普通串口          | 串口调试、`printf` 重定向 | `Hardware/Serial.c/h`               |
| 蓝牙串口          | 在线调参、遥控           | `Hardware/BlueSerial.c/h`           |
| TIM1 定时器      | 10 ms 控制任务触发      | `System/Timer.c/h`                  |
| PID 算法        | 三环控制算法            | `User/PID.c/h`                      |
| 全局应用数据        | 公共状态变量、PID实例      | `User/AppData.c/h`                  |
| FreeRTOS 任务   | 任务创建与任务函数         | `User/FreeRTOSTasks.c/h`            |
| FreeRTOS 配置   | 内核参数配置            | `User/FreeRTOSConfig.h`             |
| Hook 函数       | 堆失败、栈溢出处理         | `User/FreeRTOSHooks.c`              |
| 主程序           | 初始化与调度器启动         | `User/main.c`                       |

---

## 五、各硬件模块详细说明

## 1. MPU6050姿态检测模块

### 1.1 模块作用

MPU6050 用于测量车体运动状态，读取：

* 加速度：`AX`、`AY`、`AZ`
* 角速度：`GX`、`GY`、`GZ`

当前平衡姿态主要使用：

* `AX` 与 `AZ` 计算加速度倾角；
* `GY` 进行角速度积分。

---

### 1.2 数据读取方式

工程使用连续寄存器读取：

```c
MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
```

一次读取 14 字节，提高通信效率。

---

### 1.3 配置参数

初始化中配置：

```c
MPU6050_PWR_MGMT_1 = 0x01
MPU6050_PWR_MGMT_2 = 0x00
MPU6050_SMPLRT_DIV = 0x07
MPU6050_CONFIG = 0x00
MPU6050_GYRO_CONFIG = 0x18
MPU6050_ACCEL_CONFIG = 0x18
```

其中：

* 陀螺仪量程配置为 ±2000°/s；
* 加速度计量程配置为 ±16g。

---

### 1.4 对应工程文件

```text
Hardware/MPU6050.c
Hardware/MPU6050.h
Hardware/MPU6050_Reg.h
```

---

## 2. 软件I2C通信模块

### 2.1 模块作用

`MyI2C.c` 使用 GPIO 模拟 I2C 时序，与 MPU6050 通信。

---

### 2.2 引脚配置

```text
PB10 -> SCL
PB11 -> SDA
```

均采用：

```c
GPIO_Mode_Out_OD
```

即开漏输出。

---

### 2.3 提供的基础接口

* `MyI2C_Start()`
* `MyI2C_Stop()`
* `MyI2C_SendByte()`
* `MyI2C_ReceiveByte()`
* `MyI2C_SendAck()`
* `MyI2C_ReceiveAck()`

---

### 2.4 对应工程文件

```text
Hardware/MyI2C.c
Hardware/MyI2C.h
```

---

## 3. 电机驱动与PWM输出模块

### 3.1 模块作用

电机控制由两部分构成：

1. `Motor.c`：控制左右电机方向；
2. `PWM.c`：控制电机占空比。

---

### 3.2 电机方向引脚

```text
PB12 / PB13 -> 电机1方向
PB14 / PB15 -> 电机2方向
```

---

### 3.3 PWM输出

使用 `TIM2`：

```text
PA0 -> TIM2_CH1
PA1 -> TIM2_CH2
```

PWM 定时器参数：

```c
Period = 100 - 1
Prescaler = 36 - 1
```

输出占空比范围逻辑上对应：

```text
0 ~ 100
```

---

### 3.4 电机输出接口

```c
Motor_SetPWM(uint8_t n, int8_t PWM);
```

其中：

* `n = 1`：左侧电机；
* `n = 2`：右侧电机；
* `PWM > 0`：正转；
* `PWM < 0`：反转。

---

### 3.5 对应工程文件

```text
Hardware/Motor.c
Hardware/Motor.h
Hardware/PWM.c
Hardware/PWM.h
```

---

## 4. 编码器测速模块

### 4.1 模块作用

编码器用于测量左右轮实际转速，作为速度环和转向环反馈。

---

### 4.2 定时器资源

| 电机 | 定时器  | 引脚      |
| -- | ---- | ------- |
| 左轮 | TIM3 | PA6、PA7 |
| 右轮 | TIM4 | PB6、PB7 |

---

### 4.3 工作模式

使用 STM32 定时器硬件编码器接口模式：

```c
TIM_EncoderInterfaceConfig(...)
```

---

### 4.4 速度计算

当前工程每 50 ms 读取一次脉冲数：

```c
LeftSpeed = Encoder_Get(1) / 44.0f / 0.05f / 9.27666f;
RightSpeed = Encoder_Get(2) / 44.0f / 0.05f / 9.27666f;
```

之后计算：

```c
AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;
DifSpeed = LeftSpeed - RightSpeed;
```

---

### 4.5 对应工程文件

```text
Hardware/Encoder.c
Hardware/Encoder.h
```

---

## 5. 按键输入模块

### 5.1 模块作用

按键模块负责：

* 启动/停止小车；
* 预留扩展按键输入。

---

### 5.2 按键引脚

```text
PB1 -> Key1
PB0 -> Key2
PA5 -> Key3
PA4 -> Key4
```

使用上拉输入：

```c
GPIO_Mode_IPU
```

---

### 5.3 按键检测机制

使用 `Key_Tick()` 以固定周期运行，并采用：

* 20 次计数；
* 松手触发；

实现基础消抖与事件化读取。

---

### 5.4 当前项目中的使用方式

当前仅使用：

```text
Key1
```

实现：

* 停止 → 运行；
* 运行 → 停止。

---

### 5.5 对应工程文件

```text
Hardware/Key.c
Hardware/Key.h
```

---

## 6. LED运行状态指示模块

### 6.1 模块作用

LED 用于显示小车当前运行状态。

---

### 6.2 引脚

```text
PC13
```

---

### 6.3 控制逻辑

| RunFlag | LED |
| ------- | --- |
| 1       | 亮   |
| 0       | 灭   |

---

### 6.4 对应工程文件

```text
Hardware/LED.c
Hardware/LED.h
```

---

## 7. OLED数据显示模块

### 7.1 模块作用

OLED 用于实时显示：

* 三环 PID 参数；
* 实际角度；
* 速度反馈；
* 转向差速；
* 控制输出；
* 输出偏置。

---

### 7.2 刷新方式

刷新逻辑由：

```c
AppTask_Display()
```

统一管理，而不再放在 `main()` 主循环中。

---

### 7.3 对应工程文件

```text
Hardware/OLED.c
Hardware/OLED.h
Hardware/OLED_Data.c
Hardware/OLED_Data.h
```

---

## 8. 普通串口调试模块

### 8.1 模块作用

普通串口 `USART1` 用于：

* 基础串口调试；
* `printf` 输出重定向；
* 单字节接收测试。

---

### 8.2 串口配置

```text
USART1
PA9  -> TX
PA10 -> RX
BaudRate = 9600
```

---

### 8.3 当前工程中的状态

当前 `main()` 中会执行：

```c
Serial_Init();
```

但在正式任务流程中，普通串口并未承担核心控制功能，主要作为：

* 调试接口；
* 后续日志输出接口；
* 异常状态输出接口。

因此，它不是控制闭环的必要模块，但保留具有调试价值。

---

### 8.4 对应工程文件

```text
Hardware/Serial.c
Hardware/Serial.h
```

---

## 9. 蓝牙串口通信模块

### 9.1 模块作用

蓝牙串口用于：

* 在线修改 PID 参数；
* 发送遥控指令；
* 后续扩展自定义命令。

---

### 9.2 串口配置

```text
USART2
PA2 -> TX
PA3 -> RX
BaudRate = 9600
```

---

### 9.3 数据包协议

蓝牙串口通过中断接收，并使用：

```text
[ ... ]
```

作为完整数据包边界。

例如：

```text
[slider,AngleKp,2.4]
```

---

### 9.4 接收机制

`USART2_IRQHandler()` 中完成：

1. 判断包头 `'['`；
2. 持续接收数据；
3. 遇到包尾 `']'`；
4. 写入字符串结束符；
5. 置位：

   ```c
   BlueSerial_RxFlag = 1;
   ```

之后由 `CommTask` 解析。

---

### 9.5 对应工程文件

```text
Hardware/BlueSerial.c
Hardware/BlueSerial.h
```

---

## 10. 定时器调度模块

### 10.1 模块作用

`Timer.c` 使用 `TIM1` 产生 10 ms 周期中断，为控制任务提供稳定触发源。

---

### 10.2 参数配置

```c
TIM_Period = 10000 - 1;
TIM_Prescaler = 72 - 1;
```

在 72 MHz 时钟下：

```text
72 MHz / 72 = 1 MHz
10000 个计数 = 10 ms
```

---

### 10.3 中断优先级

```c
NVIC_IRQChannelPreemptionPriority = 5;
NVIC_IRQChannelSubPriority = 0;
```

该配置用于保证 TIM1 中断可安全调用：

```c
vTaskNotifyGiveFromISR()
```

---

### 10.4 对应工程文件

```text
System/Timer.c
System/Timer.h
```

---

## 六、控制算法说明

## 1. 三环控制结构

本工程采用：

```text
速度环 -> 角度环
转向环 -> 左右轮差速
```

的三环控制架构。

---

## 2. 姿态角估计

### 2.1 加速度计角度

```c
AngleAcc = -atan2(AX, AZ) / 3.14159f * 180.0f;
AngleAcc += 2.5f;
```

其中：

* `atan2(AX, AZ)` 用于计算倾角；
* `+2.5f` 为姿态零点修正。

---

### 2.2 陀螺仪积分角

```c
AngleGyro = Angle + GY / 32768.0f * 2000.0f * 0.01f;
```

其中：

* `2000.0f` 对应陀螺仪量程；
* `0.01f` 对应 10 ms 采样周期。

---

### 2.3 互补滤波

```c
float Alpha = 0.01f;
Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;
```

即：

* 低比例引入加速度计角度，抑制长期漂移；
* 高比例保留陀螺仪积分角，保证短时动态响应。

---

## 3. 角度环PID

### 3.1 控制目的

角度环控制车体保持直立，是系统最核心的控制环。

---

### 3.2 输入与输出

| 项目     | 内容                       |
| ------ | ------------------------ |
| 目标值    | `AnglePID.Target`        |
| 实际值    | `Angle`                  |
| 输出     | `AnglePID.Out`           |
| 电机基础输出 | `AvePWM = -AnglePID.Out` |

---

### 3.3 默认参数

```c
Kp = 2.4
Ki = 0.25
Kd = 6
OutMax = 100
OutMin = -100
OutOffset = 3
```

---

## 4. 速度环PID

### 4.1 控制目的

速度环用于控制小车前进、后退和定速运动，并将输出作为角度环目标值。

---

### 4.2 输入与输出

| 项目   | 内容                |
| ---- | ----------------- |
| 目标值  | `SpeedPID.Target` |
| 实际值  | `AveSpeed`        |
| 输出   | `SpeedPID.Out`    |
| 作用对象 | `AnglePID.Target` |

---

### 4.3 默认参数

```c
Kp = 2.2
Ki = 0.1
Kd = 0
OutMax = 20
OutMin = -20
```

---

## 5. 转向环PID

### 5.1 控制目的

转向环通过控制左右轮差速，实现小车转弯。

---

### 5.2 输入与输出

| 项目  | 内容               |
| --- | ---------------- |
| 目标值 | `TurnPID.Target` |
| 实际值 | `DifSpeed`       |
| 输出  | `DifPWM`         |

---

### 5.3 默认参数

```c
Kp = 5.5
Ki = 0.07
Kd = 0
OutMax = 50
OutMin = -50
```

---

## 6. PID通用实现

### 6.1 积分限幅

```c
if (p->ErrorInt > p->ErrorIntMax)
if (p->ErrorInt < p->ErrorIntMin)
```

用于抑制积分饱和。

---

### 6.2 微分项实现

当前微分项采用测量值变化形式：

```c
-p->Kd * (p->Actual - p->Actual1)
```

相比误差差分形式，对目标值突变的响应更平滑。

---

### 6.3 输出偏置

```c
if (p->Out > 0) { p->Out += p->OutOffset; }
if (p->Out < 0) { p->Out -= p->OutOffset; }
```

用于补偿电机死区，提高小输出时的驱动有效性。

---

## 七、FreeRTOS配置说明

## 1. 系统节拍与调度方式

### 1.1 抢占式调度

```c
#define configUSE_PREEMPTION 1
```

---

### 1.2 系统 Tick

```c
#define configTICK_RATE_HZ 1000
```

即：

```text
1 tick = 1 ms
```

---

### 1.3 最大任务优先级数量

```c
#define configMAX_PRIORITIES 6
```

当前使用优先级：

```text
5、3、2、1
```

---

## 2. 动态堆配置

### 2.1 堆大小

```c
#define configTOTAL_HEAP_SIZE ( ( size_t ) ( 10 * 1024 ) )
```

即：

```text
10 KB
```

---

### 2.2 堆管理算法

工程中使用：

```text
FreeRTOS/portable/heap_4.c
```

`heap_4` 支持内存块合并，适合一般嵌入式动态内存分配场景。

---

## 3. 中断优先级配置

### 3.1 关键宏

```c
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
```

说明：

* 中断优先级数值越小，硬件优先级越高；
* 能调用 FreeRTOS `FromISR` API 的中断，其优先级数值不能高于 5。

---

### 3.2 TIM1中断配置

```c
NVIC_IRQChannelPreemptionPriority = 5;
```

因此可调用：

```c
vTaskNotifyGiveFromISR()
```

---

### 3.3 系统异常映射

```c
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
```

用于使 FreeRTOS 接管：

* SVC；
* PendSV；
* SysTick。

---

## 4. Hook函数

### 4.1 动态内存申请失败 Hook

```c
void vApplicationMallocFailedHook(void)
```

若任务、队列等分配失败，将进入死循环。

---

### 4.2 栈溢出 Hook

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
```

若任务发生栈溢出，将停止系统继续运行。

---

## 八、主程序执行流程

`main.c` 当前执行流程如下：

```text
main()
│
├── OLED_Init()
├── MPU6050_Init()
├── BlueSerial_Init()
├── LED_Init()
├── Key_Init()
├── Motor_Init()
├── Encoder_Init()
├── Serial_Init()
├── Timer_Init()
│
├── FreeRTOSTasks_Create()
│   ├── 创建 ControlTask
│   ├── 创建 KeyLedTask
│   ├── 创建 DisplayTask
│   └── 创建 CommTask
│
├── vTaskStartScheduler()
│
└── while(1)
    └── 正常情况下不会执行到这里
```

---

## 九、蓝牙通信协议说明

### 1. 在线调参命令

格式：

```text
[slider,参数名,参数值]
```

示例：

```text
[slider,AngleKp,2.4]
[slider,AngleKi,0.25]
[slider,AngleKd,6]
[slider,SpeedKp,2.2]
[slider,TurnKi,0.07]
[slider,Offset,3]
```

---

### 2. 遥控命令

格式：

```text
[joystick,LH,LV,RH,RV]
```

示例：

```text
[joystick,0,50,25,0]
```

当前解释为：

```text
LV -> 小车前后速度目标
RH -> 小车左右转向目标
```

---

### 3. 预留按键命令

格式：

```text
[key,Name,Action]
```

当前已完成解析框架，但未绑定具体业务逻辑。

---

## 十、关键文件功能速查表

| 文件                        | 功能                        |
| ------------------------- | ------------------------- |
| `User/main.c`             | 系统初始化、任务创建、调度器启动、TIM1中断回调 |
| `User/FreeRTOSTasks.c`    | 四个核心任务的实现                 |
| `User/FreeRTOSTasks.h`    | 任务创建函数与控制任务句柄声明           |
| `User/AppData.c/h`        | 全局共享变量与三个 PID 实例          |
| `User/PID.c/h`            | 通用 PID 算法                 |
| `User/FreeRTOSConfig.h`   | FreeRTOS 核心配置             |
| `User/FreeRTOSHooks.c`    | 内存失败与栈溢出 Hook             |
| `System/Timer.c/h`        | TIM1 10 ms 定时中断           |
| `Hardware/MPU6050.c/h`    | 姿态传感器驱动                   |
| `Hardware/MyI2C.c/h`      | 软件 I2C                    |
| `Hardware/Motor.c/h`      | 电机方向控制                    |
| `Hardware/PWM.c/h`        | PWM 输出                    |
| `Hardware/Encoder.c/h`    | 编码器测速                     |
| `Hardware/OLED.c/h`       | OLED 显示驱动                 |
| `Hardware/OLED_Data.c/h`  | OLED 字库与图形数据              |
| `Hardware/Key.c/h`        | 按键输入与消抖                   |
| `Hardware/LED.c/h`        | LED 状态指示                  |
| `Hardware/Serial.c/h`     | 普通串口调试                    |
| `Hardware/BlueSerial.c/h` | 蓝牙串口接收与发送                 |

---

## 十一、当前工程特点与阶段性改造结果

### 1. 已完成的 FreeRTOS 化改造

当前工程已经完成以下重构：

| 原裸机结构       | 当前 FreeRTOS 结构   |
| ----------- | ---------------- |
| 主循环处理 OLED  | `DisplayTask`    |
| 主循环处理蓝牙解析   | `CommTask`       |
| 主循环处理按键与LED | `KeyLedTask`     |
| 中断内直接控制算法   | `ControlTask` 执行 |
| 定时器仅用于中断运算  | 定时器触发任务通知        |

---

### 2. 当前控制周期

| 功能         |     周期 |
| ---------- | -----: |
| TIM1 控制触发  |  10 ms |
| 姿态解算 + 角度环 |  10 ms |
| 速度环 + 转向环  |  50 ms |
| 按键扫描       |   1 ms |
| 蓝牙解析       |   5 ms |
| OLED 刷新    | 100 ms |

---

### 3. 当前工程的设计优势

1. **实时控制逻辑更清晰**
   控制算法从中断剥离，改由高优先级任务执行。

2. **模块职责更明确**
   控制、通信、显示、交互彼此解耦。

3. **便于后续扩展**
   可继续加入：

   * 控制状态机；
   * 故障报警；
   * 串口日志任务；
   * 队列通信；
   * 蓝牙任务通知替代轮询；
   * 运行数据记录。

4. **符合 RTOS 设计原则**
   中断短小、任务分层、优先级明确。

---

### 4. 当前仍可进一步优化的方向

1. `CommTask` 目前仍采用轮询 `BlueSerial_RxFlag`，后续可改为：

   * 二值信号量；
   * 任务通知；
   * 消息队列。

2. `Serial.c` 当前主要用于调试，若正式版本不需要串口日志，可选择：

   * 保留作调试口；
   * 或在发布版中裁剪。

3. 可将共享变量访问进一步规范化，必要时使用：

   * 临界区；
   * 互斥量；
   * 任务间消息机制。

4. 可增加系统诊断显示：

   * 剩余堆；
   * 最小剩余堆；
   * 控制节拍积压标志；
   * 任务栈高水位。

5. 可继续提升任务设计完整性：

   * 使用 `vTaskDelayUntil()` 构建周期任务；
   * 增加错误处理任务；
   * 将参数配置单独抽象为配置管理模块。

---
