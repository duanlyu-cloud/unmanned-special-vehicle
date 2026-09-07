# AR4 ROS2 项目记录

## 项目结构

```
ar4-ros2/ros2/src/
├── bringup/          # 启动文件
├── ar_description/   # URDF 模型
├── ar_gazebo/        # Gazebo 仿真
├── ar_moveit_config/ # MoveIt 配置（SRDF、运动学、规划）
├── ar_hardware_interface/ # 硬件接口
├── robot_executor/   # 运动执行封装（MoveGroupInterface）
├── task_control/     # 任务管理（调度、物料管理）
├── robot_interfaces/ # 自定义消息/服务
└── base_controller/  # 底盘控制 (v1.1 新增)
```

## 技术方案文档

| 版本 | 文件 | 说明 |
|------|------|------|
| v1.0 | `docs/v1.0_technical_plan.md` | 基础任务系统（任务区） |
| v1.1 | `docs/v1.1_technical_plan_mobile_base.md` | 移动底盘集成 |

## 关键文件

| 文件 | 说明 |
|------|------|
| `bringup/launch/gazebo_bringup.launch.py` | 主启动文件，需给 task_manager_node 传 `robot_description_semantic` |
| `task_control/config/poses.yaml` | 姿态配置（home、task_point_A/B、task_pose、物料 waypoint） |
| `robot_executor/src/robot_executor.cpp` | 运动执行，独立订阅 `/joint_states` 获取当前关节值 |
| `task_control/src/mission_scheduler.cpp` | 任务调度，构建步骤序列 |
| `task_control/src/material_manager.cpp` | YAML 配置加载 |
| `task_control/src/mission_scheduler.cpp` | 任务调度，构建步骤序列 |
| `task_control/src/material_manager.cpp` | YAML 配置加载 |

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/execute_task` | `robot_interfaces/srv/ExecuteTask` | 发送任务 |
| `/reset_state` | `std_srvs/srv/Trigger` | 重置状态（Error/Finished → Idle） |
| `/task_result` | `robot_interfaces/msg/TaskResult` | 任务结果话题 |

## 任务类型（重新定义）

- **task_type=1（放任务）**：取任务点物料 → 放回物料点
- **task_type=2（取任务）**：取物料点物料 → 放到任务点（与放任务完全镜像）

## 动作流程设计

### Task 1 — 放任务

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
8. na (关节，forward_points[0])
9. material_cartesian_path_forward (笛卡尔直线: na → material_pose)
10. material_cartesian_path_reverse (笛卡尔直线: material_pose → nc)
11. nc (关节，return_point)
12. Home (关节)
```

### Task 2 — 取任务（与 Task 1 完全镜像）

```
物料区 (6步):
1. Home (关节)
2. nc (关节，return_point)
3. material_cartesian_path_reverse反向 (笛卡尔直线: nc → material_pose)
4. material_cartesian_path_forward反向 (笛卡尔直线: material_pose → na)
5. na (关节，forward_points[0])
6. Home (关节)

任务区 (6步):
7. Home (关节)
8. task_point_B (关节)
9. task_cartesian_path_reverse反向 (笛卡尔直线: B → task_pose)
10. task_cartesian_path_forward反向 (笛卡尔直线: task_pose → A)
11. task_point_A (关节)
12. Home (关节)
```

## v1.1 系统架构（移动底盘）

```
任务区 (固定)                    物料区 (固定)
┌─────────────┐   滑轨移动    ┌─────────────────────┐
│  task_pose  │ ────────────→ │ L1  L2  L3  R1  R2  R3 │
│     ↓       │               │  ○   ○   ○   ○   ○   ○  │
│    Home     │ ←──────────── │                     │
└─────────────┘               └─────────────────────┘
```

### 任务流程

| Task | 阶段1 | 阶段2 | 阶段3 |
|------|-------|-------|-------|
| Task 1 (放) | 任务区取料 (7步) | 底盘→物料区 | 物料区放置 (7步) |
| Task 2 (取) | 物料区取料 (7步) | 底盘→任务区 | 任务区放置 (7步) |

### 左放/右放规则

| material_id | 方向 | 说明 |
|-------------|------|------|
| 1 | 左放 (L) | 使用左侧物料点 (L1) |
| 2 | 右放 (R) | 使用右侧物料点 (R1) |

## 当前完成状态

| 步骤 | 动作 | 状态 |
|------|------|------|
| 1 | Task 1 任务区 (Home→A→task_pose→B→Home) | ✓ 已完成 |
| 2 | Task 1 左放 (material_id=1, L1) | ✓ 已完成 |
| 3 | Task 1 右放 (material_id=2, R1) | ✓ 已完成 |
| 4 | Task 2 左取 (material_id=1, L1) | ✓ 已完成 |
| 5 | Task 2 右取 (material_id=2, R1) | ✓ 已完成 |

## 代码审查修复（已完成）

| 问题 | 修复 |
|------|------|
| C1 executeWithTimeout 崩溃 | 添加 `future.wait()` 防止析构崩溃 |
| C2 服务阻塞 executor | 任务改用独立线程，服务立即返回 |
| C3 YAML 错误被吞掉 | catch 块添加错误日志 |
| C4 缺失配置无提示 | 添加必填字段验证（关节数、空数组） |
| C5 占位物料数据 | 删除 material3-6，只保留 L1 和 R1 |
| H1 笛卡尔路径阈值 | 从 `<= 0.0` 改为 `< 0.90` |
| H2 `moving_` 数据竞争 | 改为 `std::atomic<bool>` |
| H3 空 return_point | 加载器和执行器都添加空检查 |
| H4 方向注释错误 | 修正：奇数=左放, 偶数=右放 |

## 真机调试状态

- 已创建 `bringup/launch/real_bringup.launch.py`（串口 `/dev/ttyUSB0`）
- Teensy 4.1 固件：`ROS2_Teensy4.1烧录固件/ROS2/ROS2.ino`
- `ar_hardware_interface` 串口连接失败时的优雅错误处理（已修复，添加 `connected_` 检查）
- **仿真验证**：Task 1/2 左右放取全部通过（100%）
- **下一步**：项目移植到香橙派（Orange Pi 5, Ubuntu ARM64）进行真机调试

## 移植计划

1. 香橙派烧录 Ubuntu 22.04 ARM64
2. 安装 ROS2 Humble + MoveIt + ros2_control
3. 用 `rsync` 同步代码到香橙派
4. 在香橙派上 `colcon build` 重新编译
5. 连接 Teensy 真机调试

## poses.yaml 结构（已实现）

```yaml
task_point_A: [j1, j2, j3, j4, j5, j6]  # Home → task_pose 中间点
task_point_B: [j1, j2, j3, j4, j5, j6]  # task_pose → Home 中间点

materials:
  material1:
    name: "Material A"
    material_pose: [j1, j2, j3, j4, j5, j6]  # 物料点位姿
    forward_points:    # 正向中间点 (从远到近: na, nb, ...)
      - [j1, j2, j3, j4, j5, j6]   # na
      - [j1, j2, j3, j4, j5, j6]   # nb
    return_point: [j1, j2, j3, j4, j5, j6]  # nc (返回用)
```

## 待解决

1. 项目移植到香橙派（Orange Pi 5, Ubuntu ARM64）
2. 在香橙派上重新编译项目
3. 连接 Teensy 真机调试
4. 在真机上验证 Task 1 和 Task 2 完整流程

## 待重构（v1.1+）

### 高影响
1. **提取解析辅助函数** - 消除 CartesianPose/JointVector 重复代码 (`material_manager.cpp`)
2. **给 loadConfig 添加错误日志** - 异常被吞掉，无错误信息 (`material_manager.cpp:131`)
3. **Step 改为 variant 设计** - 解决胖结构体问题，添加新运动类型更简单

### 中影响
4. **引入 TaskBuilder 注册表** - 替代 if-else 分发 (`mission_scheduler.cpp:49-61`)
5. **硬编码参数移入配置** - 速度、超时、规划参数等应可配置
6. **添加配置文件验证** - 检查关节向量长度、必填字段等

### 低影响
7. **修复 executeWithTimeout 线程安全** - future 生命周期管理 (`robot_executor.cpp:181`)
8. **消除 TaskManager 冗余 busy_ 状态** - 与 SchedulerState 重复
9. **物料 ID 支持显式指定** - 当前由 YAML 顺序自动分配

## 已知问题

- `robot_executor` 的 `getCurrentJointValues()` 不走 MoveGroupInterface 内部 monitor，而是独立订阅 `/joint_states`
- `task_manager_node` 需要 `MultiThreadedExecutor`
- `robot_description_semantic` 必须在 launch 中显式传给 task_manager_node
- MoveIt Humble API: `computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory)`
- `joint_state_broadcaster` 使用 `TRANSIENT_LOCAL` QoS，subscriber 必须匹配
- KDL 求解器超时需 ≥50ms，否则 `computeCartesianPath` IK 失败
- `moveCartesianPath` 调用前需等待关节状态更新（已通过 `waitForFreshJointState` 修复）
- 笛卡尔路径速度统一：使用 `setMaxVelocityScalingFactor(0.5)` 确保 A→task_pose 和 task_pose→B 速度一致

## 启动命令

```bash
cd ~/ar4-ros2/ros2
colcon build --packages-select bringup robot_executor task_control ar_moveit_config
source install/setup.bash

# 仿真模式
ros2 launch bringup gazebo_bringup.launch.py

# 真机模式（串口 /dev/ttyUSB0）
ros2 launch bringup real_bringup.launch.py
```

## 调试命令

```bash
# 发送任务
ros2 service call /execute_task robot_interfaces/srv/ExecuteTask "{task: {task_type: 1, material_id: 1, task_id: 'test_001'}}"

# 重置状态
ros2 service call /reset_state std_srvs/srv/Trigger

# 查看末端笛卡尔坐标
ros2 run tf2_ros tf2_echo base_link link_6

# 查看末端执行器位姿
ros2 topic echo /joint_states --once
```

## 关节角度范围

| 关节 | 下限 | 上限 | 说明 |
|------|------|------|------|
| joint_1 | -170° | 170° | 腰部旋转 |
| joint_2 | -42° | 90° | 肩部 |
| joint_3 | -89° | 52° | 肘部 |
| joint_4 | -165° | 165° | 腕部旋转 |
| joint_5 | -105° | 105° | 腕部俯仰 |
| joint_6 | -155° | 155° | 法兰旋转 |

## URDF Frame 名称

- 根 link: `base_link`（前面有固定 joint 连到 `world`）
- 末端 link: `link_6`（不是 `tool0`）

## 笛卡尔坐标测量结果

| 位置 | x | y | z | qx | qy | qz | qw |
|------|---|---|---|----|----|----|----|
| **Home** | -0.007 | -0.308 | 0.475 | 0 | 0.707 | -0.707 | 0 |
| **task_point_A** | -0.163 | 0.007 | 0.344 | 0.502 | 0.503 | -0.498 | -0.497 |
| **task_pose** | -0.490 | 0.006 | 0.339 | 0.502 | 0.503 | -0.498 | -0.497 |
| **task_point_B** | -0.492 | 0.006 | 0.450 | 0.502 | 0.503 | -0.498 | -0.497 |

## 当前配置

- task_pose: `[1.5679, 0.6857, -0.3811, 0, -0.2938, 0]`
- task_point_A: `[1.5679, -0.3961, 0.8943, 0, -0.4873, 0]` (Home → task_pose 中间点)
- task_point_B: `[1.5679, 0.6633, -0.8464, 0, 0.1939, 0]` (task_pose → Home 中间点)
- task_cartesian_path_forward: 已配置（task_point_A→task_pose 笛卡尔直线）
- task_cartesian_path_reverse: 已配置（task_pose→task_point_B 笛卡尔直线）
