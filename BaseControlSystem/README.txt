BaseControlSystem - BSP/APP 文件说明


===== BSP/ (板级驱动) =====

BSP\Inc\
  debug_config.h       模块调试开关 (独立启停各功能模块)
  bsp_motor.h          TB6612 电机驱动头文件
  bsp_encoder.h        霍尔编码器头文件
  bsp_lidar.h          LD06 激光雷达头文件
  bsp_uwb.h            MK8000 UWB 定位头文件
  bsp_uart.h           上位机通信协议头文件
  bsp_callbacks.h      自定义回调声明

BSP\Src\
  bsp_motor.c          TB6612 电机驱动实现
  bsp_encoder.c        霍尔编码器实现
  bsp_lidar.c          LD06 激光雷达实现
  bsp_uwb.c            MK8000 UWB 定位实现
  bsp_uart.c           上位机通信实现
  bsp_callbacks.c      HAL 回调集中实现


[b bsp_motor.c] -- TB6612 双H桥电机驱动
方向(INx GPIO) + 速度(TIM1 PWM 20kHz)
真值: INA1/INA2 = 00刹车, 10正转, 01反转, 11刹车

函数:
  Motor_Init()              启动 TIM1 CH1(左)+CH2(右) PWM (占空比 0%)
  Motor_SetLeft(duty)       左轮: >0正转, <0反转, =0刹车
  Motor_SetRight(duty)      右轮
  Motor_SetBoth(L,R)        双轮同时设（一次 GPIO, 比分开调快）
  Motor_Brake()             四路全刹 + PWM 归零


[b bsp_encoder.c] -- 霍尔编码器
TIM2(左,32位) + TIM3(右,16位) 4倍频编码器模式.
每 10ms 读 CNT -> 算 RPM -> 线速度 -> 里程mm.

可调参数: PPR=11, 减速比=90:1, 轮径=67.5mm

函数:
  Encoder_Init()                    读初始 CNT 归零, 预计算换算系数
  Encoder_Update(dt_s)              读 CNT -> 差值 -> RPM -> mm/s -> 累加里程(溢出安全)
  Encoder_GetLeftRPM()              左轮 RPM
  Encoder_GetRightRPM()             右轮 RPM
  Encoder_GetLeftSpeed_mm_s()       左轮线速度 mm/s
  Encoder_GetRightSpeed_mm_s()      右轮线速度 mm/s
  Encoder_GetLeftOdom_mm()          左轮里程 mm
  Encoder_GetRightOdom_mm()         右轮里程 mm
  Encoder_ResetOdom()               里程清零


[b bsp_lidar.c] -- LD06 激光雷达
USART2 DMA 循环接收 + IDLE 中断检测帧边界.
点云双缓冲(写Bank不收干扰,读Bank稳定).
协议: 0x54帧头, N<=12点/子帧, CRC8校验, 极坐标->直角坐标

点云字段: x(米), y(米), dist(米), angle(弧度), intensity(0-255)

函数:
  Lidar_Init()              清缓冲 -> 使能 IDLE 中断 -> 启动 DMA
  Lidar_ParseFrame(buf,len) 搜0x54 -> 校验CRC -> 角度插值 -> 极转直角 -> 写入Bank
  Lidar_GetScan(count)      取最新点云(非活跃Bank), NULL=无新数据
  Lidar_IsNewScanReady()    非阻塞查新帧(消费flag)
  Lidar_MotorOn()           雷达电机起(TIM10 30kHz 50%)
  Lidar_MotorOff()          雷达电机停
  Lidar_GetDMABuf()         返回DMA缓冲指针


[b bsp_uwb.c] -- MK8000 UWB 定位
USART1 DMA 轮询(不开中断, TIM4 每10ms Poll一次).
协议帧8B: [0xF0][0x05][addrL][addrH][distL][distH][rssi][0xAA]
测距: cm转m (uint16小端 *0.01)
滤波: EMA低通, 跳变>0.5m时降权重(0.15->0.05)
定位: 双锚点三角解算 A(0,0) B(d,0) -> x = (rA^2-rB^2+d^2)/(2d), y = sqrt(rA^2-x^2)

函数:
  UWB_Init()                清状态 -> 启动 DMA 循环接收
  UWB_Poll()                消费环缓冲 -> 逐字节解析 -> EMA滤波 -> 三角解算(x,y)
  UWB_GetState()            取最新定位结果

UWB_State_t 字段:
  anchor_A.dist_raw         A锚点原始距离(m)
  anchor_A.dist_m           A锚点滤波距离(m)
  anchor_B.*                B锚点同上
  x_m, y_m                  三角解算坐标(m)
  data_valid                1=有效 0=信号丢失
  pkt_count                 累计有效帧数

可调参数: UWB_ANCHOR_SPACING_M = 8.88m (锚点间距, 实车校准)


[b bsp_uart.c] -- 上位机通信 (USART6 115200bps, 单字节IT)
帧格式: [0xA5][CMD][LEN][载荷][XOR校验]

指令:
  0x01 载荷8B: 目标坐标 x(i32 mm) + y(i32 mm)   -> Navi_SetTarget
  0x02 载荷0B: 急停                              -> Navi_Stop
  0x03 载荷0B: 请求位姿
  0x80 载荷10B:回复位姿 x(i32)+y(i32)+yaw(i16 0.01度)

函数:
  HostUART_Init()                   启动IT接收
  HostUART_ParseByte(byte)          逐字节状态机: 等0xA5->CMD->LEN->载荷->XOR校验->分发
  HostUART_SendOdometry(x,y,yaw)    回复里程
  HostUART_SendPacket(cmd,payload,len) 通用拼帧发送


[b bsp_callbacks.c] -- 中断与业务桥梁 (TIM4 / UART / IDLE 全部在此)
  TIM4 100Hz 调用链 (各模块由 debug_config.h 独立控制):
    1. Encoder_Update(0.01f)       读编码器 -> RPM+里程
    2. UWB_Poll()                  DMA轮询->解析->三角定位
    3. HostUART_CheckTimeout()     ASCII输入超时自动提交
    4. Navi_Update()               导航调度
    5. Chassis_ControlLoop()       编码器->PID->运动学->电机
    6. LED 0.5s 闪烁              心跳

函数:
  HAL_TIM_PeriodElapsedCallback(htim)  TIM4 100Hz 主循环 (细粒度 #if 开关)
  HAL_UART_RxCpltCallback(huart)       USART1/6 IT: UWB_RxCallback / HostUART_ParseByte
  BSP_UART2_IDLE_Handler()             USART2 IDLE: 读DMA剩余->Lidar_ParseFrame


[b debug_config.h] -- 模块调试开关 (BSP\Inc\)
  每个功能模块独立编译期开关 (1=启用, 0=禁用):
    DEBUG_ENABLE_LED            LED心跳
    DEBUG_ENABLE_UWB            UWB定位
    DEBUG_ENABLE_ENCODER        编码器
    DEBUG_ENABLE_MOTOR          电机驱动
    DEBUG_ENABLE_PID            PID闭环
    DEBUG_ENABLE_NAVI           导航调度
    DEBUG_ENABLE_LIDAR          激光雷达
    DEBUG_ENABLE_HOST_UART      上位机协议
    DEBUG_ENABLE_PATH_PLAN      A*路径规划
    DEBUG_ENABLE_COORD_SOLVER   坐标解算
    DEBUG_ENABLE_MAP            超市地图

  用法: 想单测UWB → 只开 UWB+LED+HOST_UART, 其余置 0
        完整导航   → 全部置 1


===== APP/ (应用层) =====

APP\Inc\
  pid.h                PID 控制器头文件
  kinematics.h         差速运动学头文件
  chassis_task.h       底盘速度闭环头文件
  navi_task.h          导航调度头文件
  app_init.h           统一初始化头文件

APP\Src\
  pid.c                PID 控制器实现
  kinematics.c         差速运动学实现
  chassis_task.c       底盘速度闭环实现
  navi_task.c          导航调度实现
  app_init.c           统一初始化实现

APP\Header.h           聚合头文件


[a pid.c] -- 位置式 PID (浮点, 抗积分饱和, 输出限幅)
公式: u = Kp*e + Ki*积分(e*dt) + Kd*de/dt

函数:
  PID_Init(pid,Kp,Ki,Kd,dt,out_max,out_min)  初始化, 内调 Reset
  PID_SetTarget(pid,setpoint)                设目标
  PID_Compute(pid,feedback,dt_override)      一次迭代, 返回限幅后输出
  PID_Reset(pid)                             清零积分+历史误差(刹车/切模式用)


[a kinematics.c] -- 两轮差速运动学 (单位: mm, rad)
公式:
  逆解: vL = v - w*L/2,  vR = v + w*L/2
  正解: v = (vL+vR)/2,   w = (vR-vL)/L

可调参数: TRACK_WIDTH_MM = 150 (轮距)

函数:
  Kinematics_Inverse(v,w,*out)  车体(v,w) -> 左右轮速(mm/s)
  Kinematics_Forward(*in,*out)  轮速 -> 车体(v,w)


[a chassis_task.c] -- 底盘速度闭环 (10ms, TIM4驱动)
数据流: 目标(v,w) -> 逆运动学 -> 左右PID -> 占空比 -> 电机
        编码器 -> 轮速 -> 正运动学 -> 实际(v,w)

可调参数:
  WHEEL_SPEED_MAX_MM_S = 500    最大轮速(mm/s)
  SPEED_KP              = 2.0   速度环P
  SPEED_KI              = 0.5   速度环I
  SPEED_KD              = 0.02  速度环D
  SPEED_TO_DUTY_SCALE   = 10.0  mm/s->占空比(实车标定)

函数:
  Chassis_Init()           双轮PID + 编码器归零 + 电机PWM启动(占空比0,刹车)
  Chassis_SetTarget(v,w)   设目标 -> 逆运动学 -> PID目标 -> 激活
  Chassis_EmergencyStop()  刹车 + PID清零 + 关闭闭环
  Chassis_ControlLoop()    10ms: 读编码器->PID->输出->电机->正运动学
  Chassis_GetVelocity()    返回当前车体速度(遥测)


[a navi_task.c] -- 导航调度 (10ms, TIM4驱动)
数据流: 上位机发目标(x,y) -> Navi_SetTarget 存目标
        编码器 -> 里程计累加 (x,y,yaw)
        UWB -> 修正绝对 (x,y)
        Navi_Update: 算朝向+距离 -> P控制 -> v,w -> Chassis_SetTarget

可调参数:
  NAVI_ARRIVE_MM      = 50    到达判定距离(mm)
  NAVI_MAX_SPEED_MM_S = 300   最大线速度(mm/s)
  NAVI_MAX_OMEGA      = 1.5   最大角速度(rad/s)
  NAVI_K_DIST         = 0.3   距离->速度 P系数
  NAVI_K_ANGLE        = 2.0   角度差->角速度 P系数

函数:
  Navi_Init()               位姿归零
  Navi_SetTarget(x,y)       设目标坐标 -> 激活导航
  Navi_Stop()               急停 + 取消目标
  Navi_Update()             10ms: 里程推算->UWB修正->算v,w->Chassis_SetTarget
  Navi_GetPose()            获取当前位姿

特性: 大角度偏差(>57°)时自动减速30%, 先转后走.
      到达目标50mm内自动停车.
      每500ms自动上报位姿给上位机.


[a app_init.c] -- 统一初始化, main.c 只需 App_Init()

初始化顺序: 底盘 -> 导航 -> 上位机 -> UWB -> 雷达 -> TIM4(100Hz)

函数:
  App_Init()               7步拉满全部外设


[a Header.h] -- 聚合头文件
包含: HAL + Core层 + 标准C库 + 全部BSP和APP模块头文件
