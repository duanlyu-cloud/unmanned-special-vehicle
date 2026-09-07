# 改装特种车无人化装拆系统 —— 全案总体方案与上游 ROS2 实现

> © 鄂尔多斯翔天飞宇　|　本仓库为项目**上游系统（个人技术负责部分）**的公开代码库，并沉淀项目**全案总体方案**

---

## 目录

1. [项目定位与仓库范围](#1-项目定位与仓库范围)
2. [系统总体方案](#2-系统总体方案)
3. [上游系统（本仓库）](#3-上游系统本仓库)
4. [下游系统（总体方案，代码不含于本仓库）](#4-下游系统总体方案代码不含于本仓库)
5. [监控平台（Web + platform_bridge）](#5-监控平台web--platform_bridge)
6. [测试验证方法论](#6-测试验证方法论)
7. [快速开始](#7-快速开始)
8. [日常使用要点](#8-日常使用要点)
9. [真机调试要点](#9-真机调试要点)
10. [仓库目录结构](#10-仓库目录结构)
11. [项目进度与里程碑](#11-项目进度与里程碑)
12. [协议与版权](#12-协议与版权)

---

## 1. 项目定位与仓库范围

本项目为 **改装特种车（金杯车无人化装拆系统）全方案制定与上下游全链路实现**：基于金杯车底盘进行整车无人化改装，以 AR4 六轴机械臂为作业核心，集成伺服滑台底盘、三轴调平平台、视觉丝杆定位、升降装置与舵机云台等作业机构，通过 **总控 / 上游 / 下游** 三级控制系统实现"车到位 → 平台调平 → 机械臂自动装拆"的无人化作业闭环。

**仓库范围说明**：本项目按揭榜分工组织 —— 全案方案制定与上游系统实现由总设计师个人负责（即本仓库所承载的内容）；下游 STM32 固件、硬件电控与 Web 平台由团队成员按已定稿方案分工实施（代码不在本仓库）。本仓库对外公开的内容包括：

- 上游 ROS2 系统核心源码：机械臂任务调度、闭环运动控制、移动底盘伺服控制（含 ABS 智能恢复）；
- 系统全案总体方案（本章以下内容，源自《揭榜文档》v4，2026-08-13）；
- 下游系统、握手协议、监控契约的**方案与指标**（代码由对应分工角色实施）。

> 上游已实现但未包含在本公开仓库的组件（公司内部交付物）：升降台握手 ROS 服务节点、`platform_bridge`（监控桥接）、一键启停 systemd 封装、ESP8266 透明桥固件。其接口契约见下文第 4/5 章。

**项目状态快照**（截至 2026-08-07 节点，源自揭榜文档 v4）：项目总体完成度 **68%** —— 上游核心功能 100% 跑通（机械臂真机 14 步全流程、底盘往返精度实测 0.00mm）；下游分模块闭环攻关中；ESP8266 无线桥已到货烧录，**L0~L3 真机握手实测通过**。

---

## 2. 系统总体方案

### 2.1 三级控制架构

```text
┌────────────────────────────────────────────────────────────┐
│  总控层：香橙派 5 Ultra（Ubuntu 22.04 + ROS2 Humble，单机）  │
│  一键启停 / 状态聚合 / Web 监控 / 总调度                      │
└──────────────────────────┬─────────────────────────────────┘
                           │ WiFi TCP :9100（ESP8266 无线透明桥 / UART 115200 8N1）
┌──────────────────────────▼─────────────────────────────────┐
│  上游层：ROS2 三节点分布式（个人负责，本仓库代码）             │
│  机械臂任务调度 · 六轴闭环运动控制 · 底盘伺服控制与 ABS 恢复    │
│  升降台握手（上游侧） · platform_bridge 监控桥接              │
└──────────────────────────┬─────────────────────────────────┘
                           │ UART 115200 / RS-485 Modbus RTU 9600
┌──────────────────────────▼─────────────────────────────────┐
│  下游层：双 STM32F103VET6 + FreeRTOS                         │
│  主控 A：高频 IMU 动态 PID 三轴调平                          │
│  主控 B：丝杆平移（K230 视觉定位）                            │
│  末端从机（升降装置 + 舵机云台，待实施）                       │
└────────────────────────────────────────────────────────────┘
```

- **部署形态**：总控与上游主控单机部署于香橙派 5 Ultra；ROS2 工作空间 10 个包、三节点分布式架构。
- **架构原则**：方案与协议先行定稿 → 上下游按契约并行开发 → 无硬件模拟联调 → L0~L4 四层递进真机联调。

### 2.2 作业机构与模块清单

| 模块 | 机构 | 驱动 / 控制 | 方案状态 | 实施状态 | 实施分工 |
|------|------|-------------|----------|----------|----------|
| arm | AR4 六轴机械臂 | Teensy 4.1 + 编码器闭环 | ✅ 已定稿 | ✅ 上游已实现，真机 14 步全流程跑通 | 上游（本仓库） |
| chassis | PS100 伺服滑台底盘 | RS-485 Modbus RTU 位置模式 | ✅ 已定稿（v3.7） | ✅ 已实现，往返精度实测 0.00mm | 上游（本仓库） |
| lift | 升降台 | STM32 + 握手协议（TCP 行协议） | ✅ 已定稿（V1.6，L0~L3 真机通过） | ✅ 上游侧已实现；STM32 固件已实现 | 上游 / 固件 |
| leveling | 三轴调平平台 | STM32 + IMU + 增量式 PID | ✅ 已定稿 | 🚧 物理安装完成，PID 调优中（基本达标） | 方案：上游；实施：硬件电控 + 固件 |
| k230 | K230 视觉丝杆定位 | STM32 + K230 视觉（0xAA 0x55 帧协议） | ✅ 已定稿 | 🚧 识别与平移链路完成，待实车标定（基本达标） | 方案：上游；实施：固件 |
| gimbal | 升降装置 + 舵机云台（末端从机） | STM32（协议草案 LVI/KPI/GMI/STBY/ST） | 🚧 协议草案（TBD-3） | ❌ 待实施 | 下游 |
| — | 激光测距小脑（接驳筒检测） | 激光测距模块 + 导向杆 | 🚧 方案规划 | ❌ 待实施 | 下游 |

### 2.3 通信链路与网络拓扑

| 链路 | 规格 | 用途 |
|------|------|------|
| Web 监控 | REST / WebSocket，`:8080` | 监控平台，状态推送周期 1s，Web 侧零 ROS2 依赖 |
| 上下游无线握手 | WiFi TCP `:9100`，ASCII 行协议 ≤128B | 上游 ↔ STM32 升降台（经 ESP8266 无线透明桥，UART 115200 8N1） |
| 底盘伺服 | RS-485 Modbus RTU `9600` | 上游 ↔ PS100 伺服驱动器（位置模式） |
| 机械臂 | UART `115200` 8N1 | 上游 ↔ Teensy 4.1（MT/JP/JC/SS/CL 指令） |
| 下游内部 | UART/USART `115200` | IMU（USART2）、K230 视觉（USART3，PD8/PD9 全重映射） |

网络端口约定：Web `:8080`、握手 TCP `:9100`；供电与线缆规范、握手协议与监控接口契约见全案方案文档体系（公司内部，本文档为公开摘要）。

### 2.4 上下游握手协议（TCP 行协议，V1.6 / 版本 0.1.0）

- **报文格式**：ASCII 行协议，单行 ≤128B；上行指令 `STA` / `T1GO` / `T1DONE` / `T2DONE` / `ERRxx`，应答如 `STAA1B0.1.0`（协议版本 0.1.0）。
- **时序约束**：握手应答超时 5s；断线每 2s 自动重连并重新握手；无线链路延迟 10~50ms。
- **状态完整性规则**：以"状态完整性"约束上下游状态机（`WAIT_STA → READY → T1_RUNNING → READY`），杜绝半握手/状态错乱。
- **四层递进联调法（L0~L4）**：
  - L0：STM32 单板协议自测；
  - L1：传输链路（ESP8266 桥 / 网络）；
  - L2：链路握手（`STA → STAA1B0.1.0`）；
  - L3：ROS 节点联调（`/lift/handshake` OK + READY）；
  - L4：全流程（task1 → task2）。
- 无线桥方案经**双无线方案比选**后定稿：ESP8266 透明桥多客户端版 V1.1（≤4 TCP 连接互不顶断），双向透传、断线自动重连。

### 2.5 状态模型与统一错误码

- **六大模块统一状态模型**：8 状态枚举（含 STOPPED / STARTING / INITIALIZING / READY / RUNNING / ERROR 等，用于一键启停流转与监控展示）。
- **统一错误码**：8 类（覆盖超时、通信、报警、协议错误等，服务响应与 Web 共用同一套语义）。

---

## 3. 上游系统（本仓库）

### 3.1 ROS2 包架构

| Package | 角色 | 说明 |
|---------|------|------|
| `ar_description` | 机器人描述 | URDF + STL mesh（复用上游 AR4） |
| `ar_hardware_interface` | 硬件驱动 | ros2_control 硬件接口 + Teensy 串口驱动（复用上游 AR4，按需修改） |
| `ar_moveit_config` | 运动规划 | MoveIt2：SRDF、KDL 运动学、OMPL 规划器 |
| `ar_gazebo` | 仿真 | Gazebo 启动 |
| `robot_interfaces` | 接口定义 | Task / TaskResult msg，ExecuteTask / ChassisMove srv |
| `task_control` | 任务控制层 | TaskManager + MissionScheduler + MaterialManager（系统唯一状态机） |
| `robot_executor` | 动作执行层 | MoveGroupInterface 封装，无状态机，同步调用 + 超时保护 |
| `chassis_control` | 底盘控制 | PS100 伺服驱动（Modbus RTU）+ `/chassis/move_to` 服务 + ABS 恢复 |
| `bringup` | 启动 | real / gazebo 一键 launch + 参数 |

设计原则：**仅一套状态机**（MissionScheduler）；**配置与逻辑分离**（全部位姿/站点 YAML 驱动）；**同步调用模型**（顺序函数调用推进流程，便于断点调试）。

### 3.2 任务调度与状态机（task_control）

- 服务：`/execute_task`（`robot_interfaces/srv/ExecuteTask`）、`/reset_state`（`std_srvs/srv/Trigger`）。
- 任务类型（`Task.msg` 常量）：`task_type=1` = `TASK_TYPE_MATERIAL_PICK`（放料任务），`task_type=2` = `TASK_TYPE_TASK_UNLOAD`（取料任务）；参数 `material_id` 决定左/右物料位。
- 系统唯一状态机：`Idle / Running / Finished / Error`；单任务模式，Running 期间新任务直接拒绝（`accepted=false`）。
- **就绪门禁**：`joint_states` 新鲜度校验，校准期间拒绝任务下发。
- 任务编排为级联步骤序列（随版本演进：v1.0 为 12 步动作序列，v1.1 引入底盘移动后扩展；揭榜联调口径 Task1/Task2 各 **14 步**真机全流程跑通）。实际序列在 `mission_scheduler.cpp` 中按配置构建，运行时打印 `Executing N steps`，每步带日志与超时保护。
- 任务结果经 `/task_result`（`TaskResult.msg`）发布：`result_code` 0=成功 / 1=物料未找到 / 2=未知任务类型 / 3=执行失败。

### 3.3 机械臂闭环执行层（robot_executor + ar_hardware_interface）

- 动作接口：`moveHome` / `moveJoint`（moveJointPath）/ `moveCartesianPath`；末端 link `link_6`，参考系 `base_link`。
- **闭环运动控制**：编码器反馈 + 丢步自动补偿（闭环自动切换），Teensy 固件含校准流程（`REST_ENC_POSITIONS`、限位开关归零）。
- 保护机制：单步执行超时 60s（`executeWithTimeout`）；笛卡尔路径完成率 < 90% 自动重试 3 次（间隔 100ms），重试前等待最新关节状态（`waitForFreshJointState`，500ms）。
- 关键执行参数（源码为准）：

| 参数 | 当前值 |
|------|--------|
| Home/关节移动 速度、加速度缩放 | 0.5 |
| 笛卡尔路径 速度、加速度缩放 | 0.15 |
| 笛卡尔路径步长 / 最小完成度 | 0.03 m / 90%（重试 3 次） |
| 单步执行超时 | 60 s |

- 关节硬限位（URDF）：j1 ±170°、j2 −36°~96°、j3 −89°~52°、j4 ±165°、j5 ±105°、j6 ±155°（实际运动另受 MoveIt `joint_limits.yaml` 约束）。

### 3.4 移动底盘控制（chassis_control）

- 机构：PS100 伺服驱动器 + 丝杆（5mm/rev，10000 脉冲/圈），行程 0~1010mm（软限位）。
- 服务：`/chassis/move_to`（`robot_interfaces/srv/ChassisMove`，请求 `station_id` → 目标站点来自 `waypoints.yaml`；120s 超时；定位误差 ≤2mm 判成功；实测往返精度 **0.00mm**）。
- **ABS 三阶段鲁棒恢复**：重试 → P3-37 参数重初始化 → 微动唤醒；`Err29` 堵转自动恢复。
- 智能启动：断电重启 ABS 恢复秒级启动；标定文件防污染保存；站点标定（0/990mm 标定完成）。
- 关键指标：990mm 往返 ≈61s @200rpm；MVP 目标误差 ≤10mm，实际达标（≤2mm 判定，实测 0.00mm）。

### 3.5 关键接口定义（robot_interfaces）

```text
Task.msg        : uint8 task_type | uint8 material_id | string task_id
TaskResult.msg  : string task_id | uint8 result_code | string message
ExecuteTask.srv : robot_interfaces/Task task  --->  bool accepted | string message
ChassisMove.srv : string station_id            --->  bool success
                      | float64 actual_position (mm) | int32 error_code
                      (error_code: 0=OK 1=timeout 2=alarm 3=modbus_error)
                      | string message
```

| 服务 / 话题 | 类型 | 说明 |
|-------------|------|------|
| `/execute_task` | ExecuteTask | 任务下发（task_type + material_id + task_id） |
| `/reset_state` | std_srvs/Trigger | Error/Finished → Idle 重置 |
| `/chassis/move_to` | ChassisMove | 底盘移动到命名站点 |
| `/task_result` | TaskResult | 任务结果话题 |

### 3.6 姿态与站点配置（YAML 驱动）

- `task_control/config/poses.yaml`：`home_pose` / `task_point_A/B` / `task_pose` / `task_cartesian_path_forward|reverse`（笛卡尔航点）/ `materials.*`（`material_pose`、`forward_points`、`return_point`、`material_cartesian_path_forward|reverse`）。
- `chassis_control/config/waypoints.yaml`：底盘站点（task_station / material_station）。
- 全部位姿为**预标定坐标**（当前为实车实测版），修改配置无需重新编译；示教/标定方法见第 9 章。示例结构（数值仅为格式示意，非真实标定值）：

```yaml
home_pose: { joints: [0, 0, 0, 0, 0, 0] }
task_point_A: [1.5999, 0.1382, 0.5016, 0.0122, -0.6241, -0.1449]   # 2026-07-30 实测示例
task_pose:    { joints: [1.6023, 0.7251, -0.4418, 0.0242, -0.2696, -0.1151] }
materials:
  material1:
    name: "Material L1"
    material_pose: [...]      # 物料位姿（左右物料各一份）
    forward_points: [[...], ...]   # na, nb, ...
    return_point:  { joints: [...] }  # nc
```

---

## 4. 下游系统（总体方案，代码不含于本仓库）

> 下游方案由总设计师提出并定稿；STM32 固件、硬件电控与接线调试由团队成员按方案实施（分工见 2.2 表）。此处仅沉淀方案与指标摘要。

### 4.1 双 STM32 分工架构（FreeRTOS）

- **主控 A**：专职高频 IMU 动态 PID 三轴调平（USART2 采集，115200）；
- **主控 B**：负责丝杆平移（K230 视觉定位联动）。
- FreeRTOS 多任务：10ms 姿态控制任务 + 200ms 调试打印任务，Systick 1ms 基准。

### 4.2 升降台（STM32 固件 + 握手协议 V1.6）

- 协议状态机：`WAIT_STA → READY → T1_RUNNING → READY`；应答格式经 L0/L2 真机验证（`STA → STAA1B0.1.0`）。
- 与上游的联动：task1 放料 / task2 取料时序由握手协议保证（上游侧服务：`/lift/task1_send`、`/lift/task1_wait`、`/lift/task2_done`、`/lift/handshake`，状态话题 `/lift/status` —— 该组节点属公司内部交付，不在本仓库）。

### 4.3 ESP8266 无线透明桥（V1.1）

- 多客户端（≤4 TCP 连接互不顶断）；UART 115200 8N1 双向透传；断线自动重连；已烧录并实测连通（TCP :9100，L1/L2 通过）。

### 4.4 三轴调平平台（关键指标）

| 指标 | 值 |
|------|-----|
| 姿态采样率 / 控制周期 | 100Hz（10ms） |
| 姿态解算 | 卡尔曼 + 低通滤波；IMU 上电自动采集 1000 样本零偏校准（Flash 存储） |
| 控制算法 | 增量式 PID；姿态死区 0.5°；单周期输出限幅 ±20mm |
| 机构 | 3 台电机 120° 均布圆盘；三电机逆运动学解算同步步进 |
| 传动 | 400 细分 × 4mm 导程 → 脉冲当量 0.02mm/脉冲；行程 −1000~+8000 步 |
| 电机 | 默认 100Hz，31~500Hz 可调；换向冷却 50ms |
| IMU 量程 | 加速度 ±16g（1LSB≈0.488mg），陀螺 ±2000°/s |
| 姿态输出 | Roll/Pitch/Yaw 精度 0.1° |
| 保护机制 | 微动开关触碰归零 + 预升 3000 脉冲机制 |

验收口径：任意 ±15° 倾斜回正至 ±0.5° 以内。

### 4.5 K230 视觉丝杆定位（关键指标）

| 指标 | 值 |
|------|-----|
| 通讯 | USART3（PD8/PD9 全重映射），115200，0xAA 0x55 自定义帧协议 |
| 识别 | AprilTag；通讯超时 500ms 无有效帧自动切回搜索 |
| 两段式控制 | 无目标快速扫描搜索（800µs 脉冲间隔，187.5 转/分）→ 识别后慢速追踪微调（2500~8000µs，18.8~60 转/分） |
| 闭环 | 像素误差 → 速度线性映射；一阶 IIR 低通滤波（系数 0.3 可调）；追踪死区 ±8~15 像素可调 |
| 步进控制 | 梯形加减速（2µs/ms）+ DWT 微秒级脉冲（1µs 分辨率）+ 换向冷却 50ms（400 细分） |
| 追踪精度 | 追踪误差 < 15 像素 |

### 4.6 待实施模块（方案已预留）

- **下游末端从机系统**（升降装置 + 舵机云台）：协议扩展草案已提出（`LVI` / `KPI` / `GMI` / `STBY` / `ST` 命令集，行协议风格与现有协议一致），待下游评审定稿后实施；链路复用 ESP8266 透明桥，与上游经 `/lift/module_init` 服务对接。
- **激光测距小脑**：激光测距模块物理固接 + 电控实现，产出附有激光传感器的导向杆，判断挂载是否完整进入接驳筒。

---

## 5. 监控平台（Web + platform_bridge）

- **契约（V1.3 已对齐）**：REST / WebSocket，端口 `:8080`；系统状态模型 8 状态枚举；统一错误码 8 类；状态推送周期 1s。
- **platform_bridge（上游已实现，公司内部交付）**：按契约封装全部 ROS2 接口；状态聚合 JSON 周期推送；`--simulate` 模拟模式支持无硬件先行联调。
- **Web 端**：零 ROS2 依赖，纯接口对接；一键启停与单模块初始化语义见契约。
- 当前状态：platform_bridge 与 Web 待实施收尾（bridge：上游；Web：软件团队）。

---

## 6. 测试验证方法论

1. **单元/集成测试**：任务步骤序列 gtest 4 用例；底盘 ABS 恢复回归测试 pytest 13 用例。
2. **握手链路自测**：5 场景（happy / err / silent / disconnect / integrity）；TCP 链路自测。
3. **无硬件模拟联调**：STM32 模拟器（`--fail-mode`）+ bridge 模拟模式（`--simulate`），下游与 Web 侧不依赖真机即可开发联调。
4. **真机四层递进测试 L0~L4**：单板协议 → 传输链路 → 链路握手 → ROS 节点 → 全流程（task1 → task2）。
5. **实景测试**：实车参数细化与现场实测（机械臂轨迹微调、K230 识别距离二次标定、升降云台实地对接）；故障注入与自愈（拔电重连、断线重连）。

---

## 7. 快速开始

### 7.1 环境

- ROS2 Humble（Ubuntu 22.04，香橙派 5 Ultra / x86 均可）；MoveIt2、ros2_control、rosdep。
- Teensy 4.1 固件：`ros2/ROS2_Teensy4.1烧录固件/ROS2/ROS2.ino`（Arduino 环境编译烧录）。

### 7.2 构建与启动

```bash
# 构建（ros2/ 为 colcon 工作空间）
cd ros2
colcon build --packages-select bringup robot_executor task_control chassis_control ar_moveit_config
source install/setup.bash

# 仿真
ros2 launch bringup gazebo_bringup.launch.py

# 真机（serial_port: Teensy 串口，默认 /dev/ttyACM0；calibrate: 启动校准，默认 True）
ros2 launch bringup real_bringup.launch.py serial_port:=/dev/ttyACM0 calibrate:=True

# 仅底盘（PS100）
ros2 launch chassis_control chassis_bringup.launch.py
```

### 7.3 常用调试命令

```bash
# 任务下发（task_type=1 放料 / 2 取料）
ros2 service call /execute_task robot_interfaces/srv/ExecuteTask "{task: {task_type: 1, material_id: 1, task_id: 'test_001'}}"
# 状态重置
ros2 service call /reset_state std_srvs/srv/Trigger
# 底盘移动
ros2 service call /chassis/move_to robot_interfaces/srv/ChassisMove "{station_id: 'task_station'}"
# 位姿查看
ros2 run tf2_ros tf2_echo base_link link_6
ros2 topic echo /joint_states --once
```

---

## 8. 日常使用要点

- **任务下发**：`/execute_task`，参数 `task_type`（1=放料 / 2=取料，常量见 `Task.msg`）与 `material_id`（左/右物料）。任务执行期间系统拒绝新任务；异常后调用 `/reset_state` 复位。
- **校准时机**：Teensy 重新上电 / 刷写固件后必须校准；仅重启软件可 `calibrate:=False` 跳过。校准耗时 3~5 分钟。
- **就绪门禁**：关节状态未刷新或处于校准期时任务会被拒绝（`accepted=false`），属正常保护。
- **底盘 ABS**：断电重启后首次移动前触发 ABS 恢复（秒级）；`Err29` 堵转由驱动自动恢复，无需人工干预。
- **急停**：见公司内部操作规范（一键启停 systemd 单元封装，状态流转 STOPPED→STARTING→INITIALIZING→READY）。

---

## 9. 真机调试要点

| 主题 | 要点 |
|------|------|
| 编码器 | AMT-102V 为**相对编码器**，依赖限位开关确定绝对位置；限位开关弯曲或更换部件后需"精细校准" |
| 串口权限 | `sudo addgroup $USER dialout` 后注销重登 |
| Teensy 协议 | `MT` / `JP` / `ST` / `JC` / `SS` / `CL` 指令，115200bps |
| 笛卡尔路径速度 | 缩放系数统一设置（当前 0.15），避免各段速度不一致 |
| KDL 超时 | IK 求解超时设置需 ≥50ms |
| 关节状态过时 | 笛卡尔前先 `waitForFreshJointState` |
| QoS | joint_state_broadcaster 使用 `TRANSIENT_LOCAL`，订阅端必须匹配 |
| 常见故障 | 串口连接失败有 `connected_` 优雅检查；笛卡尔完成率 <90% 自动重试 |

---

## 10. 仓库目录结构

```text
├── LICENSE / README.md
├── ros2/                                   # colcon 工作空间
│   ├── src/
│   │   ├── ar_description/                 # URDF + STL（上游复用）
│   │   ├── ar_hardware_interface/          # ros2_control + Teensy 串口（上游复用）
│   │   ├── ar_moveit_config/               # MoveIt2（上游复用）
│   │   ├── ar_gazebo/                      # Gazebo 仿真（上游复用）
│   │   ├── robot_interfaces/               # Task/TaskResult/ExecuteTask/ChassisMove
│   │   ├── task_control/                   # 任务调度（任务区 + 物料区）
│   │   ├── robot_executor/                 # MoveIt2 执行封装
│   │   ├── chassis_control/                # PS100 底盘驱动 + 站点服务
│   │   └── bringup/                        # real/gazebo launch
│   ├── ROS2_Teensy4.1烧录固件/             # Teensy 4.1 固件源码
│   ├── Dockerfile / run_in_docker.sh       # 容器化（NVIDIA GPU）
└── docs/（历史文档已凝练入本 README，2026-09 起移除）
```

---

## 11. 项目进度与里程碑

> 快照来源：《揭榜文档》v4（2026-08-13，含 8.7 节点数据）；仓库代码随后持续演进。

**模块状态总览**：见 [2.2 作业机构与模块清单](#22-作业机构与模块清单)。

**MVP 核心指标对标（8.7 节点）**：

| 指标 | 目标 | 状态 |
|------|------|------|
| PS100 伺服底盘定位 | 误差 ≤10mm，0-1000mm 精准移动 | ✅ 达标（实测误差 0.00mm） |
| AR4 机械臂轨迹抓放 | 物料区抓放 80% 成功 | 🚧 接近目标（轨迹 + 固件闭环已通，待实车精调） |
| 三腿 PID 自动调平 | 姿态高频 PID 动态倾角闭环 | 🚧 基本达标（微动归零 + 3000 脉冲预升，PID 调优中） |
| K230 控制底盘/丝杆定位 | AprilTag 识别 + 近端减速微调 | 🚧 基本达标（链路完成，待实车标定） |
| 上下游无线握手通信 | Wi-Fi 无线 Task1/2 全自动闭环 | ✅ 卡点化解（ESP8266 到货烧录，TCP :9100 通，L0~L3 实测通过） |

**总体完成度**：68%（2026-08-07 节点）；上游核心功能 100% 跑通。

---

## 12. 协议与版权

- 本项目代码与文档版权归 **© 鄂尔多斯翔天飞宇** 所有（详见根目录 `LICENSE`），保留所有权利，仅供参考与学习研究，未经书面许可不得用于商业用途或再分发。
- `ros2/src` 下含上游 AR4 开源代码，遵循其自身 **MIT License**（Copyright (c) 2021 Dexter Ong），详见 `ros2/src/LICENSE`。
- 本 README 总体方案章节内容源自《揭榜文档_改装特种车全方案制定与上下游全链路实现》(v4, 2026-08-13)；技术细节与代码行为以本仓库源码及 YAML 配置为准。文档与代码不一致处已在文中标注或以代码为准。
