# AutoAim-Gimbal-Controller | 自动瞄准云台控制器

[![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-blue?style=flat-square)](./README.md)
[![MCU](https://img.shields.io/badge/MCU-STM32H7-03234B?style=flat-square)](./README.md)
[![Award](https://img.shields.io/badge/NUEDC-2nd%20Prize-success?style=flat-square)](./README.md)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square)](./LICENSE)

## 项目概述 | Project Overview

`AutoAim-Gimbal-Controller` 是 2025 年全国大学生电子设计竞赛 TI 杯获国家级二等奖作品中，二维自动瞄准云台的核心控制代码仓库。项目面向“**精准巡迹行驶 + 动态稳定瞄准**”这一复合任务场景，目标是在小车静止抖动、运动行驶与转向扰动同时存在的条件下，仍然保持激光光斑稳定、快速且准确地落在目标中心。

本仓库聚焦 **STM32H7** 主控侧的软硬件实现。系统整体采用双主控协同架构：**MSPM0** 负责小车底盘巡迹与运动控制，**STM32H7** 负责视觉数据接收、目标偏差解算、云台姿态控制、**CAN 总线**电机驱动以及基于陀螺仪的**前馈控制**补偿。对于审阅者而言，这份代码主要展示了我在 **底层硬件驱动**、**嵌入式实时控制** 与 **动态扰动抑制工程实现** 方面的能力。

From an engineering perspective, this repository highlights a complete embedded control chain for dynamic gimbal aiming: **UART visual feedback -> target error computation -> PID regulation -> CAN motor actuation -> gyroscope feedforward compensation**. Rather than presenting a conceptual demo only, the code emphasizes deployable implementation details on an STM32H7 platform.

## 系统架构与控制逻辑 | System Architecture & Control Logic

系统的功能分工如下：

- **MSPM0**: 负责底盘巡迹、行驶与基础运动控制。
- **上位机视觉端**: 通过串口发送目标框坐标信息。
- **STM32H7**: 负责串口协议解析、视觉误差解算、姿态/速度控制、双 FDCAN 电机驱动、陀螺仪扰动补偿与任务调度。
- **BMI088**: 提供运动过程中的角速度信息，用于构建动态扰动下的**前馈补偿**。

核心控制链路如下：

1. 上位机视觉端通过 **UART / USART10** 发送靶位框坐标。
2. STM32H7 在中断回调中完成 CRC8 校验与目标中心坐标提取。
3. 控制任务根据图像中心偏差进行解算，使用 **PID** 生成俯仰/方位控制量。
4. 对底部关节在动态场景中叠加 **陀螺仪角速度前馈项**，抵消小车扰动带来的瞬态偏航误差。
5. 通过 **FDCAN1 / FDCAN2** 向 DM 系列关节电机发送控制帧，实现云台快速闭环稳定。

```text
Upper Computer Vision
        |
        v
 UART / USART10  --->  CRC8 Parse & Target Center Extraction
        |                              |
        |                              v
        |                      Error Computation
        |                              |
        |                              v
        |                     PID / Feedforward Fusion
        |                              |
        v                              v
   STM32H7 RTOS Tasks  ---------->  FDCAN Motor Commands
                                       |
                                       v
                               DM Joint Motors (Yaw / Pitch)
```

![System Architecture](./docs/architecture.png)

## 核心亮点与代码导读 | Core Highlights & Code Navigation

以下部分是最值得优先审阅的代码入口，能够直接体现本项目的控制逻辑与工程实现重点，而无需陷入大量 HAL 初始化细节。

### 1. **陀螺仪前馈控制**用于动态扰动抑制

在动态行驶和转向场景下，仅依赖视觉误差闭环会出现响应滞后，因此项目将 **BMI088 陀螺仪角速度**引入控制链路作为**前馈补偿项**。在 `Mode_Motor == 3` 的动态模式中，系统读取实时角速度，并将其叠加到方位轴速度指令中，以主动抵消车体扰动。

- 关键控制入口: [`System/src/motor.c`](./System/src/motor.c)
- 传感器初始化与角速度读取: [`Hardware/src/BMI088driver.c`](./Hardware/src/BMI088driver.c)

建议优先关注 `canTask()` 中对 `BMI088_read_gy()` 的调用，以及 `speed_ctrl(&underMotor, posRingSpeed + posRingSpeed + gy/180*3.1415926);` 这一融合式控制输出。它体现了从传感器测量到执行器补偿的完整闭环工程实现。

### 2. **CAN 总线闭环电机控制**与关节驱动抽象

项目使用双路 **FDCAN** 分别驱动云台关节电机，并对 DM 系列电机协议进行了较清晰的抽象封装。底层 BSP 负责过滤器配置、中断接收和帧收发，上层驱动负责反馈解析、模式切换以及 MIT / 位置 / 速度控制指令打包。

- FDCAN 初始化与接收回调: [`Hardware/src/can_bsp.c`](./Hardware/src/can_bsp.c)
- DM 电机协议封装与控制接口: [`Hardware/src/dm_drv.c`](./Hardware/src/dm_drv.c)
- PID 输出到执行器的控制逻辑: [`System/src/motor.c`](./System/src/motor.c)
- PID 计算实现: [`System/src/pid.c`](./System/src/pid.c)

这部分代码集中体现了我对 **总线通信**, **嵌入式驱动封装**, **执行器闭环控制** 的理解与实现能力。

### 3. **双主控通信与视觉联动**的数据链路设计

本仓库虽然聚焦 STM32H7 侧，但控制链条从一开始就不是“单片机独立运行”，而是嵌入在一个双主控协同系统中。STM32H7 通过串口接收视觉系统给出的目标框信息，完成 CRC8 校验、中心点提取后，再交由控制任务进行误差解算和后续执行。

- UART 数据接收、CRC8 校验与目标中心解算: [`System/src/system.c`](./System/src/system.c)
- 系统上电初始化与任务创建入口: [`Core/Src/main.c`](./Core/Src/main.c)

其中 `HAL_UARTEx_ReceiveToIdle_IT()` 的使用，使视觉数据接收具备较好的实时性与工程适配性；`cam_calc()` 则将原始框数据转换为控制链路可直接使用的目标中心坐标。

## 代码结构速览 | Codebase Map

| Path | Role |
| --- | --- |
| [`Core/Src/main.c`](./Core/Src/main.c) | 系统启动、外设初始化、任务创建入口 |
| [`System/src/system.c`](./System/src/system.c) | 串口协议解析、CRC8 校验、任务创建、调试输出 |
| [`System/src/motor.c`](./System/src/motor.c) | 视觉误差闭环、状态机控制、前馈融合、激光触发逻辑 |
| [`System/src/pid.c`](./System/src/pid.c) | PID 控制器实现 |
| [`Hardware/src/can_bsp.c`](./Hardware/src/can_bsp.c) | FDCAN BSP、过滤器配置与中断接收 |
| [`Hardware/src/dm_drv.c`](./Hardware/src/dm_drv.c) | DM 电机协议封装与模式控制 |
| [`Hardware/src/BMI088driver.c`](./Hardware/src/BMI088driver.c) | BMI088 初始化、角速度读取与基础滤波 |
| [`MDK-ARM/project.uvprojx`](./MDK-ARM/project.uvprojx) | Keil 工程文件 |
| [`project.ioc`](./project.ioc) | STM32CubeMX 外设与时钟配置 |

## 快速开始 | Quick Start

### 编译环境

- IDE: **Keil uVision / MDK-ARM**
- Compiler: **ArmClang V6.21**
- Device Pack: **Keil.STM32H7xx_DFP 3.1.1**
- MCU: **STM32H730VBTx / STM32H730VBT6**
- Configuration Tool: **STM32CubeMX 6.15.0**

### 构建步骤

1. 使用 Keil 打开 [`MDK-ARM/project.uvprojx`](./MDK-ARM/project.uvprojx)。
2. 确认已安装对应的 **STM32H7 Device Pack** 与 ArmClang 工具链。
3. 连接目标板后编译并下载程序。
4. 若需要重新生成外设初始化代码，可使用 [`project.ioc`](./project.ioc) 在 STM32CubeMX 中打开，但请保留用户代码区中的控制逻辑实现。

### 烧录与联调注意事项

- 本工程依赖 **UART**, **FDCAN**, **SPI** 与 **FreeRTOS** 正常工作，首次上电前请确认相关外设接线正确。
- 动态前馈模式依赖 **BMI088** 返回有效角速度数据；若 IMU 初始化失败，相关补偿功能将无法生效。
- 电机和激光输出涉及实体执行机构，联调时建议先在安全状态下验证通信链路与反馈数据，再逐步开启执行器供电。

## 开发者 | Author

**Zou Yabo (邹亚博)**  
Southwest Petroleum University, Major in Internet of Things Engineering  
西南石油大学 物联网工程专业
