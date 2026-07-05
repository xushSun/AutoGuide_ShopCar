# AutoGuide ShopCar

基于 STM32F411CEU6 的超市智能导引车下位机工程。下位机接收上位机下发的目标坐标，结合 UWB 定位、MPU6050 陀螺仪、双轮编码器、LD06 雷达和静态超市地图，实现通道内路径规划、路点跟随、底盘闭环控制和辅助避障。

## 项目定位

系统整体分为三层：

- 服务端：商品数据库、Web 服务等。
- 上位机：界面、商品到坐标的映射、目标点下发。
- 下位机：本仓库，负责定位、路径规划、底盘控制、避障和串口协议。

下位机接收的是世界坐标系下的目标点 `(x_mm, y_mm)`，单位为毫米。商品查询和商品坐标映射已经从下位机中移除，由上位机处理。

## 硬件资源

- 主控：STM32F411CEU6，Cortex-M4，100 MHz。
- 电机驱动：TB6612 双 H 桥。
- 编码器：霍尔编码器，11 PPR，90:1 减速比，轮径 67.5 mm。
- 定位：MK8000 UWB，3 锚点定位。
- 陀螺仪：MPU6050，I2C1。
- 雷达：LD06，USART2 DMA circular + IDLE。
- 上位机通信：USART6，115200 bps。
- 工具链：STM32CubeMX + Keil MDK-ARM。

## 主要功能

- 上位机通过 UART 下发目标坐标。
- UWB 解析三锚测距并计算局部坐标。
- `CoordSolver` 使用 3 锚 6 参数仿射，将 UWB 局部坐标转换为超市世界坐标。
- `MapManager` 加载硬编码超市地图，包括墙、货架和通道。
- `PathPlanner` 基于 3 行 x 9 列通道路点图做 A* 搜索。
- `APP_Navigate` 使用 `IDLE -> PLAN -> ALIGN -> DRIVE -> ARRIVED` 状态机执行导航。
- 底盘采用差速模型，左右轮速度 PID 闭环。
- MPU6050 yaw 积分用于转向和直行锁航向。
- LD06 雷达提取前方、左侧、右侧最近距离，用于直行阶段辅助避障。

## 代码结构

```text
.
├── BaseControlSystem/
│   ├── APP/
│   │   ├── Inc/
│   │   └── Src/
│   │       ├── app_init.c        # 应用层统一初始化
│   │       ├── app_navigate.c    # 当前主导航状态机
│   │       ├── chassis_task.c    # 底盘速度闭环
│   │       ├── coord_solver.c    # UWB 到世界坐标仿射变换
│   │       ├── kinematics.c      # 双轮差速运动学
│   │       ├── map_manager.c     # 静态地图
│   │       ├── path_planner.c    # 通道路点图 A*
│   │       └── pid.c             # 通用 PID
│   ├── BSP/
│   │   ├── Inc/
│   │   └── Src/
│   │       ├── bsp_callbacks.c   # HAL 回调、TIM4 中断、GyroHold
│   │       ├── bsp_encoder.c     # 编码器速度和里程
│   │       ├── bsp_lidar.c       # LD06 雷达驱动
│   │       ├── bsp_motor.c       # TB6612 电机驱动
│   │       ├── bsp_uart.c        # 上位机协议
│   │       ├── bsp_uwb.c         # UWB 测距和定位
│   │       └── mpu6050.c         # MPU6050 驱动
│   ├── Core/                     # CubeMX 生成的 HAL 工程入口
│   ├── Drivers/                  # STM32 HAL/CMSIS
│   └── MDK-ARM/                  # Keil 工程
├── Tools/
│   ├── draw_map.py               # 地图说明/可视化辅助
│   └── path_plan_test.py         # 串口目标点和路点回传测试
├── UWB/                          # 早期 UWB 独立代码
├── LIDAR/                        # 早期雷达独立代码
├── map_dump.json
└── 地图.xlsx
```

> 说明：`BaseControlSystem/APP/Src/navi_task.c` 已标记为废弃，当前实际导航逻辑在 `app_navigate.c`。

## 运行时序

`main.c` 初始化 HAL 外设后调用 `App_Init()`，随后进入主循环。

主循环任务：

- 每 2 ms：读取 MPU6050，并调用 `GyroHold_Update()` 积分 yaw。
- 每 30 ms：处理 LD06 点云，更新 `g_lidar_front_m / left_m / right_m`。
- 每 50 ms：检查串口投递的目标点，并调用 `APP_Navigate_Update()`。
- 每 2 s：输出 UWB、导航、底盘、陀螺仪和雷达诊断信息。

TIM4 100 Hz 中断任务：

- `Encoder_Update(0.01f)`
- `UWB_Poll()`
- `HostUART_CheckTimeout()`
- `Chassis_ControlLoop()`
- LED 心跳

MPU6050 的 I2C 读取不放在 TIM4 ISR 中，避免阻塞中断，也避免与 `gyro_data` 产生跨上下文竞争。

## 导航策略

当前导航状态机位于 `BaseControlSystem/APP/Src/app_navigate.c`：

- `IDLE`：等待目标，底盘不动。
- `PLAN`：根据当前逻辑起点和目标点调用 `Path_Plan()`。
- `ALIGN`：原地转向到下一段路点方向，使用陀螺仪 yaw。
- `DRIVE`：直行到当前路点，使用编码器里程判断段距离，陀螺仪锁航向。
- `ARRIVED`：主动刹车并置位 `nav_arrived`。

路点规划采用通道拓扑图：

- 3 条横向通道中心线：`Y = 985, 4230, 7380`。
- 9 个列节点：`X = 600, 1110, 2220, 3330, 4440, 5550, 6660, 8800, 9270`。
- 只有最右主通道 `X = 9270` 允许南北换层，避免路径穿过货架区域。

## 上位机协议

USART6，115200 bps，二进制帧格式：

```text
[0xA5] [CMD] [LEN] [PAYLOAD...] [XOR]
```

校验：

```text
XOR = CMD ^ LEN ^ payload[0] ^ ... ^ payload[n-1]
```

常用命令：

- `0x01`：目标坐标，payload 为 `x int32 little-endian + y int32 little-endian`，单位 mm。
- `0x02`：急停。
- `0x04`：开环电机测试，payload 为左右轮百分比。
- `0x05`：设置部分锚点世界坐标。
- `0x80`：位姿回传。
- `0x90`：路点回传，payload 为 `idx + total + x + y`。

也支持简单 ASCII 输入：依次输入 `x y`，单位 m，串口解析后会转成 mm 目标点。

## 调试开关

模块开关在：

```text
BaseControlSystem/BSP/Inc/debug_config.h
```

主要开关：

- `DEBUG_ENABLE_UWB`
- `DEBUG_ENABLE_ENCODER`
- `DEBUG_ENABLE_MOTOR`
- `DEBUG_ENABLE_PID`
- `DEBUG_ENABLE_NAVI`
- `DEBUG_ENABLE_LIDAR`
- `DEBUG_ENABLE_HOST_UART`
- `DEBUG_ENABLE_PATH_PLAN`
- `DEBUG_ENABLE_COORD_SOLVER`
- `DEBUG_ENABLE_MAP`
- `DEBUG_ENABLE_GYRO`

完整导航模式通常需要开启 UWB、坐标解算、地图、路径规划、编码器、电机、PID、导航、上位机串口和陀螺仪。

## 串口测试

Python 测试脚本：

```powershell
python Tools\path_plan_test.py COM3
python Tools\path_plan_test.py COM3 5 4
```

第一条进入 ASCII 交互模式；第二条发送目标 `(5 m, 4 m)`。

## 当前版本重点

- 导航主逻辑迁移到 `app_navigate.c` 五状态机。
- 坐标转换升级为 3 锚 6 参数仿射。
- UWB 定位采用三锚 trilat2D_3A，并支持双锚回退。
- IMU 读取和 yaw 积分移出 TIM4 ISR，放入主循环。
- LD06 使用 DMA circular + IDLE + 双缓冲点云。
- 雷达避障作为直行阶段辅助，不接管主导航。

