# 基于 ROS2 的工业机械臂任务执行框架设计方案（V1.0）

---

## 文档信息

| 项目 | 内容 |
|------|------|
| 文档名称 | 基于 ROS2 的工业机械臂任务执行框架设计方案 |
| 版本 | V1.0 |
| 日期 | 2026-07-12 |
| 作者 | 吕端 |
| 状态 | 开发基线 |

---

# 1 项目概述

## 1.1 项目背景

本项目基于 **ROS2 Iron**、**MoveIt2** 与 **ros2_control**，面向固定工位工业场景，开发一套机械臂任务执行框架。系统接收执行机械臂主控下发的任务指令，自动完成物料抓取、放置及任务调度，实现固定工位自动化作业。

系统运行环境中的抓取点和放置点均采用预标定坐标配置，**V1.0 不引入视觉定位系统**，降低实现复杂度，保证运行稳定性。

## 1.2 业务场景

系统支持两种任务类型：

**Task1 — 物料抓取（MaterialPick）**

机械臂从 Home 出发，根据 `MaterialID` 移动到对应物料的抓取点，拾取物料后返回 Home，再移动到公共任务点释放物料，最后返回 Home。

```
Home → PickPose[MaterialID] → Tool Close → Home → TaskPose → Tool Open → Home
```

**Task2 — 任务点卸料（TaskUnload）**

机械臂已夹持工件，从 Home 出发移动到公共任务点拾取工件，返回 Home，再根据 `MaterialID` 移动到对应物料的放置点释放工件，最后返回 Home。

```
Home → TaskPose → Tool Close → Home → PlacePose[MaterialID] → Tool Open → Home
```

两种流程互为镜像，共用同一套动作执行接口。

## 1.3 设计目标

- **模块化**：各模块职责单一，边界清晰
- **低耦合**：模块间通过标准接口通信，不依赖内部实现
- **易维护**：代码规模可控，配置与逻辑分离
- **高扩展**：新增物料、任务类型或末端工具无需修改核心架构
- **符合 ROS2 工程规范**：充分利用 Node、Topic、Service、Action、Parameter 等通信机制

---

# 2 总体技术架构

系统采用三层架构，职责明确：**任务控制 → 动作执行 → 底层驱动**。

```text
                  执行机械臂主控
          下发任务(TaskType + MaterialID)
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    TaskControl（任务控制层）                  │
│                                                             │
│  TaskManager                                                │
│   • 接收主控下发的任务指令                                    │
│   • 校验任务参数                                             │
│   • 创建统一 Task 对象                                       │
│                                                             │
│  MaterialManager                                            │
│   • 管理 HomePose / TaskPose                                │
│   • 管理各物料 PickPose / PlacePose                         │
│   • YAML 配置驱动                                           │
│                                                             │
│  MissionScheduler（唯一状态机）                               │
│   • 任务生命周期管理                                         │
│   • Task1：物料→任务点 全流程编排                             │
│   • Task2：任务点→物料 全流程编排                             │
│   • 调用 RobotExecutor 执行动作                              │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ 调用执行动作
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                  RobotExecutor（执行层）                      │
│                                                             │
│  moveHome()       ── 返回 Home 位姿                         │
│  moveJoint()      ── 关节空间运动                           │
│  movePose()       ── 笛卡尔空间运动（预留）                   │
│  toolAction()     ── 末端工具动作                            │
│  stop()           ── 紧急停止                                │
│                                                             │
│  无状态机，仅封装 MoveIt2 调用                               │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
                     MoveIt2（运动规划）
                MoveGroupInterface
                • IK 求解  • OMPL 规划  • 碰撞检测
                          │
                          ▼
                FollowJointTrajectory Action
                          │
                          ▼
                  ros2_control 控制器
               joint_trajectory_controller
                          │
                          ▼
               ar_hardware_interface 驱动层
                   （Teensy 串口 115200bps）
                          │
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

- **仅一套状态机**：位于 `MissionScheduler`，负责完整的任务流程编排。`RobotExecutor` 无状态机，仅提供同步方法调用。
- **配置与逻辑分离**：所有位姿和物料信息由 YAML 管理，修改配置无需重新编译。
- **同步调用模型**：`MissionScheduler` 依次调用 `RobotExecutor` 的同步方法推进流程，调试时断点单步即可跟踪完整流程。

---

# 3 工程结构

基于现有 AR4 ROS2 工程增量开发，保留全部底层驱动代码，仅新增应用层 Package。

```text
robot_ws/src/

├── ar_description              （已有） 机器人 URDF 与 STL mesh
├── ar_moveit_config            （已有） MoveIt2 运动规划配置
├── ar_hardware_interface       （已有） ros2_control 硬件接口 + Teensy 驱动
├── ar_gazebo                   （已有） Gazebo 仿真启动
│
├── robot_interfaces            （新增） 自定义 msg / srv 定义
├── task_control                （新增） TaskManager + MissionScheduler + MaterialManager
├── robot_executor              （新增） RobotExecutor（MoveIt2 执行封装）
└── bringup                     （新增） 统一 Launch 文件 + YAML 配置
```

**已有 Package 说明**：

| Package | 核心内容 | 状态 |
|---------|---------|------|
| `ar_description` | URDF 宏（6 个 revolute 关节），6 个 STL mesh，ros2_control 标签宏 | 完整，不动 |
| `ar_hardware_interface` | `ARHardwareInterface`（SystemInterface），`TeensyDriver`（boost::asio 串口，115200bps，文本协议） | 完整，不动 |
| `ar_moveit_config` | SRDF（`ar_manipulator` 运动组，`home`/`upright` 命名姿态），KDL 运动学，OMPL 规划器，RViz 配置 | 完整，不动 |
| `ar_gazebo` | Gazebo 仿真 Launch | 完整，不动 |

---

# 4 核心模块设计

## 4.1 robot_interfaces

**职责**：定义系统统一通信接口，保证模块间通信标准一致。

**消息定义**：

| 名称 | 类型 | 字段 |
|------|------|------|
| `Task.msg` | 消息 | `uint8 task_type` `uint8 material_id` `string task_id` |
| `TaskResult.msg` | 消息 | `string task_id` `uint8 result_code` `string message` |

**服务定义**：

| 名称 | 类型 | 请求 | 响应 |
|------|------|------|------|
| `ExecuteTask.srv` | 服务 | `Task task` | `bool accepted` `string message` |

---

## 4.2 TaskManager

**职责**：
- 接收执行机械臂主控下发的任务指令（`TaskType` + `MaterialID`）
- 校验任务参数合法性（任务类型是否支持、物料 ID 是否在配置范围内）
- 创建 `Task` 对象
- 发布任务至 `MissionScheduler`

**任务队列机制**：

V1.0 采用**单任务模式**——系统同时仅允许一个任务执行。`TaskManager` 维护 `bool is_busy_` 标志：

- `Idle` 状态时接受新任务，置 `is_busy_ = true`，发布至 `MissionScheduler`
- `Running` 状态时收到新任务则**直接拒绝**，返回 `accepted=false`，`message="System busy, reject task: xxx"`

V2.0 可扩展为任务队列（`std::queue<Task>`），在 `MissionScheduler` 回到 `Idle` 后自动出队执行。

**输入示例**：

```
TaskType = 1（MaterialPick）
MaterialID = 3
```

**处理流程**：

```
接收指令 → 忙闲检查 → 参数校验 → 创建 Task 对象 → 发布至调度层
                       ↓（校验失败）
                   拒绝任务，返回错误码
```

---

## 4.3 MissionScheduler（唯一状态机）

**职责**：系统唯一调度中心，承担任务流程编排与生命周期管理。整个系统仅此一套状态机。

### FSM 设计

系统唯一状态机位于 `MissionScheduler`。FSM 只管理**任务生命周期**，不展开动作步骤——动作步骤是 `Running` 状态内部的顺序函数调用，不是独立状态。

**顶层 FSM（4 个状态）**：

```
             收到 Task
  ┌──────┐ ────────────→ ┌─────────┐
  │ Idle │               │ Running │
  │ 空闲  │ ←──────────── │ 执行中  │
  └──┬───┘   所有步骤完成  └────┬────┘
     │                         │
     │                         │ 任一步骤失败
     │                         ▼
     │                   ┌─────────┐
     │                   │  Error  │
     │                   │ 异常等待 │
     │                   └────┬────┘
     │                        │
     │                    人工确认恢复
     │                        │
     └────────────────────────┘
```

| 状态 | 含义 | 触发条件 |
|------|------|---------|
| `Idle` | 系统就绪，等待任务 | 启动完成 / 上一任务结束 |
| `Running` | 任务执行中，内部顺序执行动作步骤 | 收到有效 `Task` |
| `Finished` | 任务成功完成 | `Running` 中所有步骤返回成功 |
| `Error` | 执行异常，等待人工确认 | `Running` 中任一步骤返回失败 |

**"收到任务"不是状态**——它是一次性事件，`TaskManager` 发布 `Task` 后，`MissionScheduler` 立即从 `Idle` 转入 `Running`。

**"选择流程"不是状态**——它是 `Running` 入口处的一次 `switch(task_type)` 分支，瞬间完成。

### Running 内部的步骤模型

`Running` 不是一个静止状态，而是一个**状态 + 内部循环**。步骤序列如下：

```
buildSteps(task)
      │
      ▼
┌─────────────────────────────────────────────────┐
│               Running（步骤循环）                │
│                                                 │
│  Step 1: GoHome                                 │
│  Step 2: MoveTo(target1)   ← target1 由分支决定  │
│  Step 3: Tool Close                            │
│  Step 4: GoHome                                │
│  Step 5: MoveTo(target2)   ← target2 由分支决定  │
│  Step 6: Tool Open                             │
│  Step 7: GoHome                                │
│                                                 │
│  任一步骤返回 false → 立即跳出 → 转入 Error       │
│  全部步骤返回 true  → 转入 Finished              │
└─────────────────────────────────────────────────┘
```

每个 Step 就是一个**结构体**：`{ 调用哪个 RobotExecutor 方法, 传入什么目标位姿 }`。

**步骤序列定义**：

```cpp
enum class ActionType { Home, MoveTo, ToolClose, ToolOpen };

struct Step {
    ActionType action;
    std::vector<double> target;  // MoveTo 时使用，其他动作忽略
};
```

两种任务的步骤**结构完全相同**，差异仅在于 `MoveTo` 的 `target` 参数：

```
Task1（MaterialPick）:
  Home → MoveTo(PickPose) → ToolClose → Home → MoveTo(TaskPose) → ToolOpen → Home

Task2（TaskUnload）:
  Home → MoveTo(TaskPose) → ToolClose → Home → MoveTo(PlacePose) → ToolOpen → Home
```

**伪代码**：

```cpp
void MissionScheduler::executeTask(const Task& task) {
    state_ = State::Running;
    auto material = material_manager_.getMaterial(task.material_id);

    // 构建步骤列表：Task1 和 Task2 共用同一组 Action，仅目标位姿不同
    std::vector<Step> steps = (task.task_type == TASK_TYPE_MATERIAL_PICK)
        ? buildTask1Steps(material)    // PickPose → TaskPose
        : buildTask2Steps(material);   // TaskPose → PlacePose

    // 顺序执行，任一步失败即中止
    for (const auto& step : steps) {
        if (!executeStep(step)) {
            state_ = State::Error;
            return;
        }
    }

    state_ = State::Finished;
}

bool MissionScheduler::executeStep(const Step& step) {
    switch (step.action) {
        case ActionType::Home:      return executor_.moveHome();
        case ActionType::MoveTo:    return executor_.moveJoint(step.target);
        case ActionType::ToolClose: return executor_.toolClose();
        case ActionType::ToolOpen:  return executor_.toolOpen();
    }
    return false;
}
```

**关键设计约束**：
- FSM 只描述任务生命周期（Idle / Running / Finished / Error），不描述动作步骤
- 动作步骤是 `Running` 内部的 `Step` 序列，通过 `for` 循环顺序执行
- `MissionScheduler` 不直接调用 MoveIt2 API，所有运动通过 `RobotExecutor` 的同步方法
- 新增任务类型：只需新增一个 `buildTask3Steps()` 方法，返回一组 `Step`
- `RobotExecutor` 无状态机，每个方法为同步阻塞调用，返回 `bool`

---

## 4.4 MaterialManager

**职责**：管理所有物料信息与系统固定位姿。配置与逻辑完全分离。

**数据结构**：

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

  material2:
    name: "物料B"
    pick_pose:
      joints: [0.3, 0.2, 0.6, 0.0, 1.0, 0.0]
    place_pose:
      joints: [0.4, 0.3, 0.7, 0.0, 1.1, 0.0]

  material3:
    name: "物料C"
    pick_pose:
      joints: [0.5, 0.0, 0.5, 0.0, 0.8, 0.0]
    place_pose:
      joints: [0.6, 0.0, 0.6, 0.0, 0.9, 0.0]
```

**内部数据结构**：

```cpp
struct JointPose {
    std::vector<double> joints;  // 6 关节角度 (rad)
};

struct Material {
    uint8_t id;
    std::string name;
    JointPose pick_pose;
    JointPose place_pose;
    double tool_open_angle;   // 可选，0 表示使用全局默认值
    double tool_close_angle;  // 可选，0 表示使用全局默认值
};

class MaterialManager {
public:
    bool loadConfig(const std::string& yaml_path);
    std::optional<Material> getMaterial(uint8_t material_id) const;
    JointPose getHomePose() const;
    JointPose getTaskPose() const;
    double getToolOpenAngle(uint8_t material_id = 0) const;
    double getToolCloseAngle(uint8_t material_id = 0) const;
private:
    JointPose home_pose_;
    JointPose task_pose_;
    double tool_open_angle_;   // 全局默认
    double tool_close_angle_;  // 全局默认
    std::map<uint8_t, Material> materials_;
};
```

**设计原则**：
- 每种物料同时保存 `pick_pose` 和 `place_pose`，Task1 和 Task2 统一使用同一份配置
- Task1 使用 `pick_pose`，Task2 使用 `place_pose`
- 工具角度支持全局默认值（`tool.open_angle` / `tool.close_angle`），也支持按物料覆盖（`material.tool_open_angle` / `tool_close_angle`），物料未配置时回退到全局默认值
- 新增物料仅增加 YAML 条目，无需修改代码
- `getMaterial()` 返回 `std::optional`，物料 ID 不存在时由调用方处理

---

## 4.5 RobotExecutor

**职责**：封装 MoveIt2 调用，提供统一动作接口。不含业务逻辑，不含状态机。

**设计特点**：
- **无状态机**：每个方法为同步阻塞调用，返回 `bool` 表示成功/失败
- **单一职责**：仅负责"可靠地执行一个动作"，不关心动作序列和流程编排
- **内部超时保护**：每个方法内置超时机制，防止 MoveIt2 规划或执行永久阻塞

**接口定义**：

```cpp
class RobotExecutor {
public:
    explicit RobotExecutor(rclcpp::Node::SharedPtr node);

    // 初始化（加载 MoveGroupInterface）
    bool init();

    // 基础运动
    bool moveHome();                                  // 回 Home（SRDF named target）
    bool moveJoint(const std::vector<double>& joints); // 关节空间运动
    bool movePose(const geometry_msgs::msg::Pose& pose); // 笛卡尔空间运动（预留）

    // 末端工具（V1.0 通过 joint_6 角度模拟）
    bool toolOpen();
    bool toolClose();

    // 紧急停止
    void stop();

    // 查询状态
    bool isMoving() const;

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    std::chrono::milliseconds timeout_{10000};  // 默认 10 秒超时
};
```

**关键实现要点**：

```cpp
bool RobotExecutor::moveJoint(const std::vector<double>& joints) {
    move_group_->setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        return false;
    }

    auto result = move_group_->execute(plan);
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool RobotExecutor::moveHome() {
    move_group_->setNamedTarget("home");
    auto result = move_group_->move();  // plan + execute in one call
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool RobotExecutor::toolClose() {
    // V1.0: 通过 joint_6 旋转至闭合角度模拟
    // close_angle 从 MaterialManager 的 YAML 配置获取，非硬编码
    auto joints = move_group_->getCurrentJointValues();
    joints[5] = close_angle_;  // joint_6
    return moveJoint(joints);
}
```

**MoveIt2 超时保护实现**：

MoveIt2 的 `move()` 和 `execute()` 是阻塞调用，规划失败或硬件无响应时可能长时间卡死。每个运动方法内部通过 `std::async` + `std::future::wait_for()` 实现超时保护：

```cpp
// RobotExecutor 内部辅助方法：带超时的动作执行
bool RobotExecutor::executeWithTimeout(
    std::function<bool()> action,
    const std::string& action_name)
{
    auto future = std::async(std::launch::async, action);

    if (future.wait_for(timeout_) == std::future_status::timeout) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Timeout after %ld ms — cancelling and stopping",
            action_name.c_str(), timeout_.count());

        stop();  // 取消当前 MoveIt2 Action
        return false;
    }

    try {
        bool result = future.get();
        if (!result) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Execution failed", action_name.c_str());
        }
        return result;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Exception: %s", action_name.c_str(), e.what());
        return false;
    }
}

// moveJoint 使用超时包装
bool RobotExecutor::moveJoint(const std::vector<double>& joints) {
    return executeWithTimeout([&]() {
        move_group_->setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            return false;
        }
        return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }, "moveJoint");
}
```

> **注意**：`stop()` 通过 MoveIt2 的 Action 取消接口停止当前运动，适用于软件层面。硬件级急停（Teensy 固件 `ES` 指令）属于 V2.0 范围。

**运动模式对照**：

| 方法 | 目标类型 | MoveIt2 接口 | 用途 |
|------|---------|-------------|------|
| `moveHome()` | 命名姿态 | `setNamedTarget("home")` | 回归初始位姿（复用 SRDF 已有定义） |
| `moveJoint(joints)` | 关节角度 | `setJointValueTarget(joints)` | 移动到预定义工位点 |
| `movePose(pose)` | 笛卡尔位姿 | `setPoseTarget(pose)` | V2.0 视觉引导场景预留 |
| `toolOpen()` | 关节角度 | `setJointValueTarget({..., open_angle})` | joint_6 模拟末端打开 |
| `toolClose()` | 关节角度 | `setJointValueTarget({..., close_angle})` | joint_6 模拟末端闭合 |

所有方法最终均通过 `FollowJointTrajectory` Action 执行。

**V2.0 扩展**：增加独立末端执行器后，仅需修改 `toolOpen()` / `toolClose()` 内部实现，`MissionScheduler` 调用接口不变。

---

# 5 ROS2 通信架构

| 通信链路 | 通信方式 | 说明 |
|---------|---------|------|
| 执行机械臂主控 → TaskManager | Topic / Service | 接收外部任务指令 |
| TaskManager → MissionScheduler | Topic | 发布 `Task` 消息 |
| MissionScheduler → MaterialManager | C++ 接口（同进程） | 查询物料位姿，零通信开销 |
| MissionScheduler → RobotExecutor | C++ 接口（同进程） | `moveJoint()` / `toolOpen()` 等同步调用 |
| RobotExecutor → MoveIt2 | `MoveGroupInterface` | MoveIt2 C++ API |
| MoveIt2 → ros2_control | `FollowJointTrajectory` Action | 标准轨迹执行接口 |
| ros2_control → Hardware | `SystemInterface::read()` / `write()` | 100Hz 实时循环 |
| Hardware → Teensy | 串口文本协议（115200bps） | `MT` / `JP` / `ST` / `JC` / `SS` 指令 |

---

# 6 数据配置

所有位姿和物料参数采用 YAML 配置，修改无需重新编译。

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

  material2:
    name: "物料B"
    pick_pose:
      joints: [0.3, 0.2, 0.6, 0.0, 1.0, 0.0]
    place_pose:
      joints: [0.4, 0.3, 0.7, 0.0, 1.1, 0.0]

  material3:
    name: "物料C"
    pick_pose:
      joints: [0.5, 0.0, 0.5, 0.0, 0.8, 0.0]
    place_pose:
      joints: [0.6, 0.0, 0.6, 0.0, 0.9, 0.0]
```

---

# 7 非功能设计

## 7.1 日志

统一使用 `RCLCPP_INFO` / `RCLCPP_WARN` / `RCLCPP_ERROR` 宏。记录以下关键事件：

- 任务接收（TaskType + MaterialID + 时间戳）
- 每个动作执行结果（规划成功/失败、执行耗时）
- 任务完成或异常终止
- 异常与错误信息

## 7.2 异常处理

`MissionScheduler` 的 `handleError()` 中：

1. 立即调用 `executor_.stop()` 停止当前运动
2. 记录错误日志（失败的动作、当前关节角度）
3. 转入 `Error` 状态，等待人工确认恢复
4. V2.0：支持自动重试与恢复策略

**错误恢复方式**：

V1.0 通过 ROS2 Service 实现人工确认恢复。`MissionScheduler` 在 `Error` 状态下暴露 `/reset_error` 服务：

- 服务类型：`std_srvs::srv::Trigger`（ROS2 标准服务，无请求参数，返回 success + message）
- 调用方式：命令行 `ros2 service call /reset_error std_srvs/srv/Trigger` 或上位机程序调用
- 处理逻辑：收到请求后，`MissionScheduler` 清除错误标记，复位至 `Idle`，等待新任务

```cpp
// MissionScheduler 内部
reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "/reset_error",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (state_ == State::Error) {
            state_ = State::Idle;
            res->success = true;
            res->message = "Error cleared, back to Idle";
            RCLCPP_INFO(node_->get_logger(), "Error reset by user");
        } else {
            res->success = false;
            res->message = "Not in Error state";
        }
    });
```

使用 `std_srvs::srv::Trigger` 无需自定义 srv 定义，降低接口复杂度。

## 7.3 急停

V1.0 在 `RobotExecutor` 中提供 `stop()` 方法，调用 MoveIt2 的 Action 取消接口停止当前运动。

V2.0 增加 Teensy 固件 `ES`（Emergency Stop）串口指令——Teensy 收到后立即将所有 AccelStepper 目标置为当前位置并停止脉冲输出，实现硬件级急停。

## 7.4 超时保护

`RobotExecutor` 的每个运动方法内部通过 `std::future` + `wait_for()` 实现超时保护。MoveIt2 规划或执行超过设定时间（默认 10 秒）时自动取消并返回失败，防止系统死锁。

---

# 8 硬件适配说明

## 8.1 已复用现有模块

| 模块 | 路径 | 状态 |
|------|------|------|
| URDF 机器人描述 | `ar_description/urdf/ar_macro.xacro` | 6 个 revolute 关节，完整惯性/碰撞/视觉属性 |
| SRDF 语义描述 | `ar_moveit_config/srdf/ar_macro.srdf.xacro` | `ar_manipulator` 运动组，`home`/`upright` 命名姿态 |
| ros2_control 接口 | `ar_hardware_interface` | SystemInterface，position 控制，joint offset 校准 |
| Teensy 串口驱动 | `ar_hardware_interface/src/teensy_driver.cpp` | boost::asio，115200bps，MT/JP/ST/JC/SS 协议 |
| Teensy 固件 | `ROS2.ino` | 6 路 AccelStepper + Encoder（开环），限位开关校准 |
| MoveIt2 配置 | `ar_moveit_config` | KDL 运动学，OMPL 规划，FollowJointTrajectory Action |
| Gazebo 仿真 | `ar_gazebo` | gazebo_ros2_control 插件 |

## 8.2 当前硬件限制

### （1）固定工位

所有位姿采用预标定 YAML 配置，不依赖视觉定位。

### （2）末端执行器

AR4 当前未安装独立夹爪。V1.0 的 `toolOpen()` / `toolClose()` 通过将 `joint_6` 旋转至预设角度模拟末端工具动作。

V2.0 增加独立末端执行器后，仅需修改 `RobotExecutor` 中这两个方法的内部实现。

### （3）运动反馈

当前 Teensy 固件中编码器实际读数被注释（[ROS2.ino L106](ros2/ROS2_Teensy4.1烧录固件/ROS2/ROS2.ino#L106)），位置反馈来自 AccelStepper 内部计数器，系统运行于开环模式。步进电机失步时 ROS2 端无法感知实际位置偏差。

V2.0 可启用编码器真实读数，构建闭环控制。

---

# 9 V2.0 架构预留（不纳入本期实施）

以下功能已在架构设计阶段预留扩展接口，**V1.0 不进行实现**，但无需对核心框架进行重构即可接入。

| 序号 | 扩展方向 | 接入方式 | 影响范围 |
|------|---------|---------|---------|
| ① | **独立末端执行器**（电动夹爪/真空吸盘/气动夹具/快换工具） | 替换 `RobotExecutor::toolOpen()` / `toolClose()` 内部实现 | `robot_executor` 内部 |
| ② | **视觉定位系统** | 新增 `VisionManager`，通过 `RobotExecutor::movePose()` 注入目标位姿 | 新增模块 |
| ③ | **多机械臂协同** | 多个 `RobotExecutor` 实例 + `MissionScheduler` 增加多臂调度策略 | `task_control` 内部 |
| ④ | **Behavior Tree** | 将 `MissionScheduler::executeTask()` 的线性流程迁移至 BehaviorTree.CPP | `task_control` 内部 |
| ⑤ | **MES/ERP 对接**（OPC UA / MQTT / REST API） | `TaskManager` 增加工业通信适配层 | `task_control` 内部 |
| ⑥ | **闭环运动控制**（启用编码器反馈） | 修改 Teensy 固件 + `ar_hardware_interface` | 驱动层 |
| ⑦ | **数字孪生**（Gazebo / Isaac Sim） | 基于已有 `ar_gazebo` 扩展 | 独立模块 |
| ⑧ | **AI 智能调度** | 新增 `TaskOptimizer`，注入 `MissionScheduler` | 新增模块 |

---

# 10 方案优势

| 对比维度 | 本方案 | 传统单文件+全局状态机 |
|---------|--------|---------------------|
| 架构设计 | 三层分离，任务控制/动作执行/底层驱动各司其职 | 逻辑混杂，难以维护 |
| 状态机数量 | **仅一套**，位于 MissionScheduler | 隐式分布在多个回调中 |
| 模块耦合 | 低耦合，RobotExecutor 可独立测试 | 全局变量依赖，无法单独测试 |
| 扩展新增物料 | 仅增加 YAML 条目 | 修改主循环中的 case 分支 |
| 扩展新增任务类型 | 在 executeTask() 增加分支，约 7 行代码 | 需要重构状态机 |
| 扩展新增末端工具 | 仅修改 RobotExecutor 内部实现 | 贯穿全局的散落修改 |
| 现有代码复用 | 100% 复用已有 4 个 Package | 推倒重来或强行嵌入 |
| ROS2 规范 | Node / Parameter / Action / MoveGroupInterface | 仅使用 Topic |
| 调试能力 | 同步调用，断点单步即可跟踪完整流程 | 异步回调，状态分散 |
| 学习门槛 | 简单的 FSM + 同步方法调用 | 需理解多层异步回调 |

---

# 11 实施路线

| 阶段 | 内容 | 预计时间 |
|------|------|---------|
| **Phase 1** | `robot_interfaces` — Task.msg / TaskResult.msg / ExecuteTask.srv 定义 | 0.5 天 |
| **Phase 2** | `robot_executor` — RobotExecutor（moveHome / moveJoint / toolOpen / toolClose / stop） | 2 天 |
| **Phase 3** | `task_control` — TaskManager + MaterialManager + MissionScheduler（唯一 FSM） | 2.5 天 |
| **Phase 4** | `bringup` — 统一 Launch + poses.yaml 配置 + 集成调试 | 1 天 |
| **Phase 5** | 真机联调 + 异常场景测试 + 文档 | 2 天 |

---

# 12 结论

本方案以现有 AR4 ROS2 工程为基础，充分复用底层硬件驱动（`ar_hardware_interface`）、运动规划（MoveIt2 / OMPL / KDL）和仿真环境（`ar_gazebo`），仅在应用层新增 `task_control`、`robot_executor` 和 `robot_interfaces` 三个 Package，避免重复开发，降低实施风险。

整体架构采用 **TaskManager → MissionScheduler(FSM) → RobotExecutor** 的三层执行模型：

- **整个系统仅一套状态机**，位于 `MissionScheduler`，统一管理任务全生命周期
- **MissionScheduler** 直接编排任务流程：根据 `TaskType` 和 `MaterialID` 参数，依次调用 `RobotExecutor` 的同步方法推进动作序列
- **RobotExecutor** 不含状态机与业务逻辑，仅封装 MoveIt2 的 `MoveGroupInterface`，提供 `moveHome()` / `moveJoint()` / `toolOpen()` / `toolClose()` 等同步方法
- **MaterialManager** 管理所有位姿与物料配置，采用 YAML 驱动，新增物料零代码改动
- **MoveIt2** 负责 IK 求解与 OMPL 运动规划，通过 `FollowJointTrajectory` Action 交由 ros2_control 执行

V1.0 聚焦固定工位、预标定坐标和稳定任务执行，采用单一状态机 + 同步调用模型，对 ROS2 初学者友好，调试路径清晰；V2.0 预留视觉定位、独立末端执行器、多机械臂协同、Behavior Tree、MES 集成和闭环控制等扩展方向。通过三层分离的模块化设计，系统能够在不重构核心框架的前提下持续演进。

---

*文档版本：V1.0 | 日期：2026-07-12 | 状态：开发基线*
