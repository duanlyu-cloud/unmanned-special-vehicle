# AR4 六轴机械臂 ROS2 无人化任务执行系统

> © 鄂尔多斯翔天飞宇

本项目基于 **ROS2**、**MoveIt2** 与 **ros2_control**，以 AR4 六轴机械臂为核心，面向固定工位工业场景，开发一套机械臂任务执行框架。系统接收执行机械臂主控下发的任务指令，自动完成物料抓取、放置及任务调度，实现固定工位自动化作业。v1.1 起集成移动底盘（滑轨），在任务区与物料区之间往返，实现无人化任务执行。

抓取点与放置点均采用预标定坐标配置，**当前版本不引入视觉定位系统**，以降低实现复杂度、保证运行稳定性。

---

## 1. 项目简介

- **机械臂**：AR4 六轴机械臂（Annin Robotics），6 个 revolute 关节。
- **控制框架**：ROS2 + MoveIt2 + ros2_control。
- **任务模式**：接收主控下发的任务指令（`TaskType` + `MaterialID`），自动完成物料抓取、放置与调度。
- **移动底盘（v1.1）**：滑轨式移动底盘，在任务区与物料区之间往返，实现无人化任务执行。
- **定位方式**：预标定坐标（YAML 配置），不依赖视觉。

---

## 2. 系统组成

### 2.1 硬件

| 部件 | 说明 |
|------|------|
| AR4 六轴机械臂 | 6 个 revolute 关节，URDF 描述 + STL mesh |
| Teensy 4.1 | 电机控制器固件（`ROS2.ino`），6 路 AccelStepper + 编码器（开环），限位开关校准 |
| Arduino Nano | 辅助控制器（桌面应用 IO 控制） |
| PS100 伺服底盘（v1.1） | 滑轨式移动底盘，伺服电机驱动 |
| 传感器 | 关节编码器（AMT-102V 相对编码器）、限位开关 |

> 说明：当前 Teensy 固件中编码器实际读数被注释，位置反馈来自 AccelStepper 内部计数器，系统运行于**开环模式**。步进电机失步时 ROS2 端无法感知实际位置偏差。

### 2.2 软件（ROS2 包）

| Package | 角色 |
|---------|------|
| `ar_description` | 机器人 URDF 描述与 STL 模型（已有，复用） |
| `ar_hardware_interface` | ros2_control 硬件接口 + Teensy 串口驱动（已有，复用） |
| `ar_moveit_config` | MoveIt2 运动规划配置（SRDF、KDL 运动学、OMPL 规划器）（已有，复用） |
| `ar_gazebo` | Gazebo 仿真启动（已有，复用） |
| `robot_interfaces` | 自定义 msg / srv 定义（新增） |
| `task_control` | 任务管理：TaskManager + MissionScheduler + MaterialManager（新增） |
| `robot_executor` | 运动执行封装（MoveGroupInterface）（新增） |
| `bringup` | 统一 Launch 文件 + YAML 配置（新增） |
| `chassis_control` / `base_controller` | 底盘控制（v1.1 新增） |

---

## 3. 软件架构与任务执行框架

### 3.1 三层架构

系统采用三层架构，职责明确：**任务控制 → 动作执行 → 底层驱动**。

```text
执行机械臂主控
   下发任务(TaskType + MaterialID)
          │
          ▼
┌─────────────────────────────────────────────┐
│            TaskControl（任务控制层）          │
│  TaskManager     接收/校验任务，创建 Task     │
│  MaterialManager 管理位姿与物料（YAML 驱动）  │
│  MissionScheduler 唯一状态机，流程编排        │
└─────────────────────────────────────────────┘
          │ 调用执行动作
          ▼
┌─────────────────────────────────────────────┐
│        RobotExecutor（动作执行层）            │
│  moveHome / moveJoint / movePose /          │
│  toolAction / stop                          │
│  无状态机，仅封装 MoveIt2 调用               │
└─────────────────────────────────────────────┘
          ▼
     MoveIt2（运动规划）
     MoveGroupInterface（IK / OMPL / 碰撞检测）
          ▼
     FollowJointTrajectory Action
          ▼
     ros2_control（joint_trajectory_controller）
          ▼
     ar_hardware_interface（Teensy 串口 115200bps）
          ▼
     AR4 六轴机械臂
```

**各层职责**：

| 层级 | 职责 | 对应模块 |
|------|------|---------|
| 任务控制层 | 任务接收与校验、物料位姿管理、流程编排与调度 | `TaskManager` + `MaterialManager` + `MissionScheduler` |
| 动作执行层 | 封装 MoveIt2 调用，提供统一动作接口，不含业务逻辑 | `RobotExecutor` |
| 底层驱动层 | 运动规划、轨迹执行、硬件通信 | MoveIt2 / ros2_control / `ar_hardware_interface` |

**架构设计原则**：

- **仅一套状态机**：位于 `MissionScheduler`，负责完整任务流程编排。`RobotExecutor` 无状态机，仅提供同步方法调用。
- **配置与逻辑分离**：所有位姿和物料信息由 YAML 管理，修改配置无需重新编译。
- **同步调用模型**：`MissionScheduler` 依次调用 `RobotExecutor` 的同步方法推进流程，调试时断点单步即可跟踪完整流程。

### 3.2 核心模块交互

```
task_manager_node (入口)
    ↓
TaskManager (服务层) ←→ ROS2 Service (/execute_task, /reset_state)
    ↓
MissionScheduler (调度层) → Step 序列构建 + 执行
    ↓
RobotExecutor (执行层) → MoveGroupInterface 封装
    ↑
MaterialManager (数据层) ← YAML 配置
```

- **TaskManager**：接收主控下发的任务指令，校验参数合法性（任务类型是否支持、物料 ID 是否在配置范围内），创建 `Task` 对象并发布至 `MissionScheduler`。V1.0 采用**单任务模式**，系统同时仅允许一个任务执行；`Running` 状态时收到新任务直接拒绝（`accepted=false`）。
- **MissionScheduler**：系统唯一调度中心，承担任务流程编排与生命周期管理。FSM 只管理任务生命周期（`Idle` / `Running` / `Finished` / `Error`），动作步骤是 `Running` 状态内部的顺序函数调用。
- **MaterialManager**：管理所有物料信息与系统固定位姿，YAML 配置驱动。每种物料同时保存 `pick_pose` 和 `place_pose`，Task1 使用 `pick_pose`，Task2 使用 `place_pose`。
- **RobotExecutor**：封装 MoveIt2 调用，提供统一动作接口。无状态机，每个方法为同步阻塞调用，返回 `bool`。内置超时保护（默认 10 秒），防止 MoveIt2 规划或执行永久阻塞。

### 3.3 关键接口

**消息定义**：

| 名称 | 类型 | 字段 |
|------|------|------|
| `Task.msg` | 消息 | `uint8 task_type` `uint8 material_id` `string task_id` |
| `TaskResult.msg` | 消息 | `string task_id` `uint8 result_code` `string message` |

`task_type` 常量（定义于 `Task.msg`）：`1`= `TASK_TYPE_MATERIAL_PICK`（取料），`2`= `TASK_TYPE_TASK_UNLOAD`（卸料）。

`TaskResult.result_code`：`0`=成功，`1`=物料未找到，`2`=未知任务类型，`3`=执行失败。

**服务定义**：

| 名称 | 类型 | 请求 | 响应 |
|------|------|------|------|
| `ExecuteTask.srv` | 服务 | `Task task` | `bool accepted` `string message` |
| `ChassisMove.srv` | 服务（v1.1） | `string station_id`（目标站点，定义于 waypoints.yaml） | `bool success` `float64 actual_position`(mm) `int32 error_code`(0=OK/1=timeout/2=alarm/3=modbus_error) `string message` |

**其他服务/话题**：

| 名称 | 类型 | 说明 |
|------|------|------|
| `/execute_task` | `robot_interfaces/srv/ExecuteTask` | 发送任务 |
| `/reset_state` | `std_srvs/srv/Trigger` | 重置状态（Error/Finished → Idle） |
| `/task_result` | `robot_interfaces/msg/TaskResult` | 任务结果话题 |

> 注：设计方案 V1.0 中错误恢复服务名为 `/reset_error`，实际实现（AGENTS.md / v1.0 技术方案）使用 `/reset_state`。两者均为 `std_srvs/srv/Trigger`。

### 3.4 任务类型与动作流程

**任务类型**（代码常量见 `Task.msg`；设计文档曾以"放任务 / 取任务"命名，方向语义请以真机验证为准）：

- **task_type=1（`TASK_TYPE_MATERIAL_PICK`，文档称"放任务"）**：任务区与物料区往返动作，完整流程如下。
- **task_type=2（`TASK_TYPE_TASK_UNLOAD`，文档称"取任务"）**：与任务 1 完全镜像。

**Task 1 — 取料任务（12 步）**：

```
任务区 (6步):
1. Home (关节)
2. task_point_A (关节)
3. task_cartesian_path_forward (笛卡尔直线: A → task_pose)
4. task_cartesian_path_reverse (笛卡尔直线: task_pose → B)
5. task_point_B (关节)
6. Home (关节)

物料区 (6步，material_id 决定左右):
7. Home (关节)
8. 沿 forward_points 逐点关节移动 (na, nb, …)
9. material_cartesian_path_forward (笛卡尔直线: 末航点 → material_pose)
10. material_cartesian_path_reverse (笛卡尔直线: material_pose → return_point)
11. return_point (关节, nc)
12. Home (关节)
```

> 注：任务区/物料区的具体航点（task_point_A/B、na/nb/nc、material_pose、task_pose）与夹爪动作时机均以 `task_control/config/poses.yaml` 及 `mission_scheduler.cpp` 实际代码为准，上图仅为流程结构示意。

**Task 2 — 卸料任务（与 Task 1 完全镜像，12 步）**：

```
物料区 (6步):
1. Home (关节)
2. return_point (关节, nc)
3. material_cartesian_path_reverse 反向 (笛卡尔直线: nc → material_pose)
4. material_cartesian_path_forward 反向 (笛卡尔直线: material_pose → 航点)
5. 沿 forward_points 反向逐点关节移动 (…, nb, na)
6. Home (关节)

任务区 (6步):
7. Home (关节)
8. task_point_B (关节)
9. task_cartesian_path_reverse 反向 (笛卡尔直线: B → task_pose)
10. task_cartesian_path_forward 反向 (笛卡尔直线: task_pose → A)
11. task_point_A (关节)
12. Home (关节)
```

> 注：设计方案 V1.0 中任务流程为 `Home → PickPose → ToolClose → Home → TaskPose → ToolOpen → Home`（Task1 物料抓取）与 `Home → TaskPose → ToolClose → Home → PlacePose → ToolOpen → Home`（Task2 任务点卸料），与上述 12 步流程在动作语义上一致，但步骤划分与命名不同。实际实现以 12 步流程为准。

### 3.5 姿态配置文件（poses.yaml）

所有位姿和物料参数采用 YAML 配置，修改无需重新编译。结构如下：

```yaml
# task_control/config/poses.yaml
home_pose:
  joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

task_pose:
  joints: [0.5, -0.3, 0.8, 0.0, 1.2, 0.0]

# 末端工具参数（V1.0 通过 joint_6 角度模拟）
tool:
  open_angle: 0.0    # joint_6 打开角度 (rad)
  close_angle: 1.57  # joint_6 闭合角度 (rad)

materials:
  material1:
    name: "物料A"
    pick_pose:
      joints: [0.3, -0.5, 0.6, 0.0, 1.0, 0.0]
    place_pose:
      joints: [0.4, -0.4, 0.7, 0.0, 1.1, 0.0]
    tool_open_angle: 0.0    # 可选，覆盖全局默认值
    tool_close_angle: 1.57  # 可选，覆盖全局默认值
```

实际实现（v1.0 技术方案）中，任务区与物料区使用 `task_point_A` / `task_point_B` / `task_pose` / `task_cartesian_path_forward` / `task_cartesian_path_reverse`，物料使用 `material_pose` / `forward_points`（na, nb, ...）/ `return_point`（nc）/ `material_cartesian_path_forward` / `material_cartesian_path_reverse`。

> ⚠️ 上表 yaml 中的数值仅为**格式示意**，非真实标定值。真机标定坐标以 `task_control/config/poses.yaml` 为准（当前为 2026-07-30 实测版本），代码中相关配置示例：`task_point_A: [1.5999, 0.1382, 0.5016, 0.0122, -0.6241, -0.1449]`、`task_point_B: [1.6062, 0.7449, -1.0642, 0.0057, 0.1703, -0.1432]`、`task_pose.joints: [1.6023, 0.7251, -0.4418, 0.0242, -0.2696, -0.1151]`。标定值会随现场调试更新，修改配置无需重新编译。

---

## 4. 快速开始

### 4.1 环境

- **ROS2 发行版**：设计方案 V1.0 与上游 `src/README.md` 标注为 **ROS2 Iron（Ubuntu 22.04）**；移植计划中目标平台为香橙派（Orange Pi 5, Ubuntu ARM64）上的 **ROS2 Humble**。请以实际部署环境为准。
- 依赖：MoveIt2、ros2_control、rosdepc 工具。

### 4.2 构建

```bash
# 在仓库 ros2/ 目录（colcon 工作空间根）下执行
colcon build --packages-select bringup robot_executor task_control ar_moveit_config
source install/setup.bash
```

### 4.3 启动

**仿真模式**：

```bash
ros2 launch bringup gazebo_bringup.launch.py
```

**真机模式**：

```bash
# serial_port: Teensy 4.1 串口，默认 /dev/ttyACM0；calibrate: 启动时是否校准，默认 True
# 按实际硬件接线可用 serial_port:=/dev/ttyUSB0 覆盖
ros2 launch bringup real_bringup.launch.py serial_port:=/dev/ttyACM0 calibrate:=True
```

> 注：设计方案 V1.0 与目录结构文档中规划的一键启动文件为 `bringup/launch/system.launch.py`，实际实现使用 `gazebo_bringup.launch.py` 与 `real_bringup.launch.py`。

**上游 AR4 驱动启动参数**（`ar_hardware_interface`）：

- `calibrate`：是否对机械臂进行校准（确定每个关节的绝对位置）。
- `include_gripper`：是否包含伺服夹爪，默认 `True`。
- `serial_port`：Teensy 板串口，默认 `/dev/ttyACM0`。
- `arduino_serial_port`：Arduino Nano 板串口，默认 `/dev/ttyUSB0`。

> 注：真机调试记录中 `real_bringup.launch.py` 曾使用串口 `/dev/ttyUSB0`，与上游默认值（Teensy 为 `/dev/ttyACM0`）存在差异，请按实际硬件连接配置。

**MoveIt 演示**（RViz 中规划，不含真实/模拟臂）：

```bash
ros2 launch ar_moveit_config demo.launch.py
```

**Gazebo 仿真 + MoveIt**：

```bash
ros2 launch ar_gazebo ar_gazebo.launch.py
ros2 launch ar_moveit_config ar_moveit.launch.py use_sim_time:=true include_gripper:=True
```

### 4.4 Docker（可选）

`ros2/` 目录提供 `Dockerfile` 与 `run_in_docker.sh`。需要 NVIDIA GPU 及 NVIDIA 容器工具包：

```bash
docker build -t ar4_ros_driver .
./run_in_docker.sh
```

### 4.5 调试命令

```bash
# 发送任务 1（task_type=1: 取料 MATERIAL_PICK）
ros2 service call /execute_task robot_interfaces/srv/ExecuteTask "{task: {task_type: 1, material_id: 1, task_id: 'test_001'}}"

# 发送任务 2（task_type=2: 卸料 TASK_UNLOAD）
ros2 service call /execute_task robot_interfaces/srv/ExecuteTask "{task: {task_type: 2, material_id: 1, task_id: 'test_002'}}"

# 重置状态
ros2 service call /reset_state std_srvs/srv/Trigger

# 查看末端笛卡尔坐标
ros2 run tf2_ros tf2_echo base_link link_6

# 查看关节状态
ros2 topic echo /joint_states --once
```

---

## 5. 日常使用要点

> 以下内容摘自《使用手册 v1.2》。该手册主体为上游 AR3/AR4 桌面控制应用（ARCS）与 AR3 ROS1 环境的操作说明，与本项目 ROS2 框架相关的要点如下。

### 5.1 通讯

- 确定 Teensy 4.1 与 Arduino Nano 的通信端口（Windows 设备管理器或 Arduino IDE 工具菜单 → 端口），在设备"端口号"输入字段中设置。只需设置一次，软件会记住 COM 端口。

### 5.2 速度 / 加速 / 减速

- 机器人速度设置为最大速度的百分比。速度 100 为最快。低速典型值在 **10% 到 25%** 之间。
- 加速和减速各有 2 个参数：**持续时间**（移动百分比）与**百分比**（幅度度量）。例如 100mm 移动，前 5mm 内快速加速可设 Dur=5；最后 25mm 内缓慢停止可设减速 Dur=25。

### 5.3 运行（点动）

- 在"点动度数"框输入移动度数，按对应"-"或"+"按钮移动每个关节。
- 关节模式：可选中"步进关节"单选按钮按电机步进点动。
- 笛卡尔坐标点动：输入移动距离（毫米），按"-"或"+"点动。
- 工具坐标点动：根据夹具微动。
- Xbox 控制器慢跑：3 种模式（关节、笛卡尔、重定向），D 垫控制方向，X 键切换关节组，A 键切笛卡尔，B 键控制方向，Y 键示教位置，开始按钮开/关第一个 DO（典型用于开闭夹持器）。

### 5.4 编程（桌面应用）

- **Move J**：关节移动，所有关节共同完成的扫掠运动，最简单常用。
- **Move L**：线性移动，执行完美直线到示教位置。
- **Move A**：弧形移动，需示教 3 个点（起点 / 中点 / 终点）。
- **Move C**：圆形移动，需示教 3 个点（中心 / 起点 / 平面点）。
- **Move SP**：存储位置，寄存器选项卡可设置 16 个存储位置（X, Y, Z, Y, P, R）。
- **OFFS SP**：移动到存储位置并偏移另一存储位置的值，适合按行堆叠放置。
- **Teach SP**：将当前位置存储到存储位置寄存器。
- 暂停：等待时间 / 等待输入 / 等待输入关闭。
- IO：设置输出打开/关闭（Arduino 可用 IO 引脚 14-19）。
- 导航：可创建多个程序例程，支持调用程序 / 返回 / 创建选项卡 / 跳转到选项卡 / If Register Jump。
- 寄存器：设置静态值或递增（`++1`）/ 递减（`--1`）。

### 5.5 校准（桌面应用）

- **强制校准**：强制在每个轴的中点校准，仅在构建和设置期间使用。
- **精细校准**：设置参考位置以检查校准真实性（弯曲限位开关或更换部件后）。示教参考位置 → 转到精校准位置检查精度 → 小步点动校正 → 执行精校准。
- **方向默认值**：校准默认设置在安装限位开关的轴的一侧，6 个值（每关节一个），只能为"0"或"1"。
- **机器人校准值**：输入每个关节的运动自由度及步进电机步数。

### 5.6 任务下发与坐标概念

- 任务通过 `/execute_task` 服务下发，参数为 `task_type`（1=放任务，2=取任务）与 `material_id`。
- 系统位姿（Home、task_point_A/B、task_pose、物料位姿）均为预标定坐标，配置于 `poses.yaml`。
- 末端 link 为 `link_6`（不是 `tool0`），根 link 为 `base_link`。

---

## 6. 真机调试要点

### 6.1 校准流程

- 向 Teensy 板刷新固件，以及为机械臂 / Teensy 板断电重新上电后，**需要进行校准**。
- 校准可通过启动参数 `calibrate:=False` 跳过（仅重启软件但未重启 Teensy 时，编码器保持供电，无需重新校准）。
- 原始编码器（AMT-102V）为**相对编码器**，上电时需对照限位开关校准。
- 也可修改 Teensy 固件中的 `REST_ENC_POSITIONS` 值，使编码器在启动时初始化为该值（前提是机械臂始终在该位置初始化）。
- 开始任何运动前，建议在 RViz 中检查模型是否处于合理位置，以验证编码器校准正确。

### 6.2 编码器与限位开关检查

- 编码器为相对编码器，依赖限位开关确定绝对位置。
- 限位开关弯曲或更换机械部件后，需通过"精细校准"重新验证校准真实性。
- 当前固件编码器实际读数被注释，位置反馈来自 AccelStepper 内部计数器（开环模式）。

### 6.3 串口 / Teensy 注意事项

- Teensy 4.1 固件：`ROS2_Teensy4.1烧录固件/ROS2/ROS2.ino`。
- `ar_hardware_interface` 串口连接失败时已有优雅错误处理（`connected_` 检查）。
- 串口通信协议：`MT` / `JP` / `ST` / `JC` / `SS` 指令，115200bps。
- 若未启用串口访问，需执行 `sudo addgroup $USER dialout` 并注销重新登录。

### 6.4 常见问题与解决方案

| 问题 | 解决方案 |
|------|----------|
| 笛卡尔路径速度不一致 | 笛卡尔移动统一通过 `setMaxVelocityScalingFactor` / `setMaxAccelerationScalingFactor` 设置（当前 0.15） |
| KDL 求解器超时 | 超时设置需 ≥50ms，否则 `computeCartesianPath` IK 失败 |
| 关节状态过时 | `moveCartesianPath` 调用前等待关节状态更新（`waitForFreshJointState`） |
| joint_state_broadcaster QoS 不匹配 | 使用 `TRANSIENT_LOCAL` QoS，subscriber 必须匹配 |
| 串口连接失败 | `ar_hardware_interface` 已添加 `connected_` 检查，优雅处理 |

### 6.5 关节角度范围

| 关节 | 下限 | 上限 | 说明 |
|------|------|------|------|
| joint_1 | -170° | 170° | 腰部旋转 |
| joint_2 | -36° | 96° | 肩部 |
| joint_3 | -89° | 52° | 肘部 |
| joint_4 | -165° | 165° | 腕部旋转 |
| joint_5 | -105° | 105° | 腕部俯仰 |
| joint_6 | -155° | 155° | 法兰旋转 |

> 注：以上限位取自 `ar_description/urdf/ar_macro.xacro`，为 URDF 中的硬限位；实际运动还受 MoveIt 配置（`joint_limits.yaml`）约束。

### 6.6 运动执行参数

| 参数 | 当前值 | 来源 |
|------|--------|------|
| Home / 关节移动 速度与加速度缩放 | 0.5 | `robot_executor.cpp` |
| 笛卡尔路径 速度与加速度缩放 | 0.15 | `robot_executor.cpp` `moveCartesianPath` |
| 笛卡尔路径步长 | 0.03 m | `computeCartesianPath` |
| 笛卡尔最小完成度 | 90%（不足则重试，最多 3 次，间隔 100ms） | `robot_executor.cpp` |
| 单步执行超时 | 60 s | `RobotExecutor::timeout_` |
| 笛卡尔前关节状态新鲜度等待 | 500 ms | `waitForFreshJointState` |

---

## 7. 目录结构（简版）

```text
arm/
├── ros2/                                    # ROS2 工作空间根目录
│   ├── src/
│   │   ├── ar_description/                  # （已有）机器人描述（URDF + STL mesh）
│   │   ├── ar_hardware_interface/           # （已有）硬件驱动（ros2_control + Teensy 串口）
│   │   ├── ar_moveit_config/                # （已有）MoveIt2 配置
│   │   ├── ar_gazebo/                       # （已有）Gazebo 仿真
│   │   ├── robot_interfaces/                # （新增）msg / srv 定义
│   │   │   ├── msg/  (Task.msg, TaskResult.msg)
│   │   │   └── srv/  (ExecuteTask.srv)
│   │   ├── task_control/                    # （新增）任务控制
│   │   │   ├── config/poses.yaml            # Home / TaskPose / Materials
│   │   │   └── src/  (task_manager_node.cpp, mission_scheduler.cpp, material_manager.cpp)
│   │   ├── robot_executor/                  # （新增）执行层（MoveIt2 封装）
│   │   └── bringup/                         # （新增）启动
│   │       └── launch/                      # gazebo_bringup.launch.py / real_bringup.launch.py
│   ├── ROS2_Teensy4.1烧录固件/              # （已有）Teensy 固件（ROS2/ROS2.ino）
│   ├── Dockerfile                           # （已有）
│   └── run_in_docker.sh                     # （已有）
└── ar4_4.3.2应用程序/                       # （已有）桌面控制应用，与 ROS2 无关
```

---

## 8. 版本与技术计划摘要

### v1.0 — 基础任务系统（任务区）

- 三层架构：任务控制 → 动作执行 → 底层驱动。
- 新增 `robot_interfaces`、`task_control`、`robot_executor`、`bringup` 四个 Package，复用已有 4 个底层 Package。
- 单任务模式，唯一状态机位于 `MissionScheduler`。
- 末端工具通过 `joint_6` 角度模拟（`toolOpen()` / `toolClose()`）。
- 仿真验证：Task 1/2 左右放取全部通过（100%）。

**当前完成状态**：

| 步骤 | 动作 | 状态 |
|------|------|------|
| 1 | Task 1 任务区（Home→A→task_pose→B→Home） | ✓ 已完成 |
| 2 | Task 1 左放（material_id=1, L1） | ✓ 已完成 |
| 3 | Task 1 右放（material_id=2, R1） | ✓ 已完成 |
| 4 | Task 2 左取（material_id=1, L1） | ✓ 已完成 |
| 5 | Task 2 右取（material_id=2, R1） | ✓ 已完成 |

### v1.1 — 移动底盘集成

- 新增 `BaseExecutor`（底盘执行层）与 `base_controller` / `chassis_control` 包。
- 任务流程扩展为三阶段：任务区取料 → 底盘移动 → 物料区放置（15 步）。
- `ActionType` 扩展 `MoveBase`，`Step` 结构体扩展 `BaseTarget`。
- `ExecuteTask.srv` / `Task.msg` 扩展 `material_position_id` 字段。
- 底盘控制话题：`/base_controller/cmd_pos`、`/base_controller/pos`、`/base_controller/enable`、`/base_controller/stop`。
- 左放/右放规则：v1.1 方案中**奇数 ID（1,3,5）为右放（R），偶数 ID（2,4,6）为左放（L）**。

> ⚠️ **左右规则冲突**：AGENTS.md 中记录 `material_id=1` 为左放（L）、`material_id=2` 为右放（R），而其代码审查修复 H4 又注明"奇数=左放, 偶数=右放"，与 v1.1 方案（奇数=右放, 偶数=左放）相互矛盾。请以实际代码与真机验证结果为准。

### 当前状态与下一步

- 仿真验证 Task 1/2 全部通过。
- 下一步：项目移植到香橙派（Orange Pi 5, Ubuntu ARM64）进行真机调试。
- 移植步骤：烧录 Ubuntu 22.04 ARM64 → 安装 ROS2 Humble + MoveIt + ros2_control → `rsync` 同步代码 → `colcon build` 重新编译 → 连接 Teensy 真机调试。

### V2.0 预留扩展方向（不纳入本期实施）

| 扩展方向 | 接入方式 |
|---------|---------|
| 独立末端执行器（电动夹爪/真空吸盘/气动夹具/快换工具） | 替换 `RobotExecutor::toolOpen()` / `toolClose()` 内部实现 |
| 视觉定位系统 | 新增 `VisionManager`，通过 `movePose()` 注入目标位姿 |
| 多机械臂协同 | 多个 `RobotExecutor` 实例 + 多臂调度策略 |
| Behavior Tree | 将 `executeTask()` 线性流程迁移至 BehaviorTree.CPP |
| MES/ERP 对接（OPC UA / MQTT / REST API） | `TaskManager` 增加工业通信适配层 |
| 闭环运动控制（启用编码器反馈） | 修改 Teensy 固件 + `ar_hardware_interface` |
| 数字孪生（Gazebo / Isaac Sim） | 基于已有 `ar_gazebo` 扩展 |
| AI 智能调度 | 新增 `TaskOptimizer`，注入 `MissionScheduler` |

---

## 9. 协议与版权

- 本项目代码版权归 **© 鄂尔多斯翔天飞宇** 所有（详见根目录 `LICENSE`），保留所有权利，仅供参考与学习使用，未经书面许可不得用于商业用途或再分发。
- `ros2/src` 目录下包含上游 AR4 代码，遵循其自身的 **MIT License**（Copyright (c) 2021 Dexter Ong），详见 `ros2/src/LICENSE`。

---

> 本 README 由原 `docs/` 目录下多份项目文档凝练而成（使用手册 / 架构方案 / 技术计划 / 真机调试 / 目录结构等）。具体行为以源码与 YAML 配置为准；文档与代码不一致处已在文中标注。
