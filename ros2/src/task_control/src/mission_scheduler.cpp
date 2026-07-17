#include "task_control/mission_scheduler.hpp"

namespace task_control {

static geometry_msgs::msg::Pose cartesianToPose(const CartesianPose &cp) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = cp.x;
  pose.position.y = cp.y;
  pose.position.z = cp.z;
  pose.orientation.x = cp.qx;
  pose.orientation.y = cp.qy;
  pose.orientation.z = cp.qz;
  pose.orientation.w = cp.qw;
  return pose;
}

MissionScheduler::MissionScheduler(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<robot_executor::RobotExecutor> executor,
    const MaterialManager &material_manager)
    : node_(node), executor_(executor), material_manager_(material_manager) {
  result_pub_ =
      node_->create_publisher<robot_interfaces::msg::TaskResult>("/task_result",
                                                                  10);
}

void MissionScheduler::executeTask(uint8_t task_type, uint8_t material_id,
                                   const std::string &task_id) {
  if (state_ == SchedulerState::Running) {
    RCLCPP_WARN(node_->get_logger(), "Mission already running, rejecting");
    return;
  }

  state_ = SchedulerState::Running;
  last_result_ = "Running task: " + task_id;

  auto material_opt = material_manager_.getMaterial(material_id);
  if (!material_opt) {
    RCLCPP_ERROR(node_->get_logger(), "Material %d not found", material_id);
    publishResult(task_id, 1, "Material not found");
    state_ = SchedulerState::Error;
    last_result_ = "Material not found";
    return;
  }

  auto material = material_opt.value();
  std::vector<Step> steps;

  if (task_type == 1) {
    // 放任务: 任务点 → 物料点
    steps = buildTask1Steps(material);
  } else if (task_type == 2) {
    // 取任务: 物料点 → 任务点
    steps = buildTask2Steps(material);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown task type: %d", task_type);
    publishResult(task_id, 2, "Unknown task type");
    state_ = SchedulerState::Error;
    last_result_ = "Unknown task type";
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Executing %zu steps", steps.size());

  for (size_t i = 0; i < steps.size(); ++i) {
    const auto &step = steps[i];
    const char* action_names[] = {"Home", "MoveTo", "MovePath", "MoveCartesian"};
    RCLCPP_INFO(node_->get_logger(), "Step %zu/%zu: %s", i + 1, steps.size(),
                action_names[static_cast<int>(step.action)]);

    if (!executeStep(step)) {
      executor_->stop();
      publishResult(task_id, 3, "Step execution failed");
      state_ = SchedulerState::Error;
      last_result_ = "Step execution failed";
      RCLCPP_ERROR(node_->get_logger(), "Step %zu failed, entering Error state", i + 1);
      return;
    }
  }

  publishResult(task_id, 0, "Task completed");
  state_ = SchedulerState::Idle;
  last_result_ = "Task completed: " + task_id;
  RCLCPP_INFO(node_->get_logger(), "Task completed: %s", task_id.c_str());
}

SchedulerState MissionScheduler::getState() const { return state_; }

void MissionScheduler::resetState() {
  if (state_ == SchedulerState::Error) {
    state_ = SchedulerState::Idle;
    last_result_ = "State reset to Idle";
    RCLCPP_INFO(node_->get_logger(), "State reset to Idle");
  }
}

std::string MissionScheduler::getLastResult() const { return last_result_; }

void MissionScheduler::publishResult(const std::string &task_id,
                                     uint8_t result_code,
                                     const std::string &message) {
  robot_interfaces::msg::TaskResult msg;
  msg.task_id = task_id;
  msg.result_code = result_code;
  msg.message = message;
  result_pub_->publish(msg);
}

// ============================================================================
// Task 1: 放任务 (取任务点物料 → 放回物料点)
// 流程: 任务点部分 → 物料点部分
// ============================================================================
std::vector<Step>
MissionScheduler::buildTask1Steps(const Material &material) {
  std::vector<Step> steps;

  // 任务点部分: Home → A → task_pose → B → Home
  auto task_steps = buildTaskPointForwardSteps();
  steps.insert(steps.end(), task_steps.begin(), task_steps.end());

  // 物料点部分: Home → na → nb → ... → material_pose → nc → Home
  auto material_steps = buildMaterialForwardSteps(material);
  steps.insert(steps.end(), material_steps.begin(), material_steps.end());

  return steps;
}

// ============================================================================
// Task 2: 取任务 (取物料点物料 → 放到任务点)
// 流程: 物料点部分 → 任务点部分
// ============================================================================
std::vector<Step>
MissionScheduler::buildTask2Steps(const Material &material) {
  std::vector<Step> steps;

  // 物料点部分: Home → nc → material_pose → nb → ... → na → Home
  auto material_steps = buildMaterialReturnSteps(material);
  steps.insert(steps.end(), material_steps.begin(), material_steps.end());

  // 任务点部分: Home → B → task_pose → A → Home
  auto task_steps = buildTaskPointReturnSteps();
  steps.insert(steps.end(), task_steps.begin(), task_steps.end());

  return steps;
}

// ============================================================================
// 任务点部分步骤
// ============================================================================

// Home → task_point_A → (笛卡尔) → task_pose → (笛卡尔) → task_point_B → Home
std::vector<Step> MissionScheduler::buildTaskPointForwardSteps() {
  std::vector<Step> steps;

  // 1. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  // 2. task_point_A (关节空间)
  auto task_point_a = material_manager_.getTaskPointA();
  if (!task_point_a.empty()) {
    steps.push_back({ActionType::MoveTo, task_point_a, {}, {}});
  }

  // 3. task_cartesian_path_forward (笛卡尔直线: A → task_pose)
  auto task_cartesian_fwd = material_manager_.getTaskCartesianPathForward();
  if (!task_cartesian_fwd.empty()) {
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto &cp : task_cartesian_fwd) {
      poses.push_back(cartesianToPose(cp));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, poses});
  }

  // 4. task_cartesian_path_reverse (笛卡尔直线: task_pose → B)
  auto task_cartesian_rev = material_manager_.getTaskCartesianPathReverse();
  RCLCPP_INFO(node_->get_logger(), "task_cartesian_path_reverse size: %zu",
              task_cartesian_rev.size());
  if (!task_cartesian_rev.empty()) {
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto &cp : task_cartesian_rev) {
      poses.push_back(cartesianToPose(cp));
      RCLCPP_INFO(node_->get_logger(), "  reverse wp: [%.3f, %.3f, %.3f]",
                  cp.x, cp.y, cp.z);
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, poses});
  }

  // 5. task_point_B (关节空间)
  auto task_point_b = material_manager_.getTaskPointB();
  if (!task_point_b.empty()) {
    steps.push_back({ActionType::MoveTo, task_point_b, {}, {}});
  }

  // 6. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  return steps;
}

// task_pose → (笛卡尔) → task_point_B → Home (用于 Task 2 返回)
std::vector<Step> MissionScheduler::buildTaskPointReturnSteps() {
  std::vector<Step> steps;

  // 1. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  // 2. task_point_B (关节空间)
  auto task_point_b = material_manager_.getTaskPointB();
  if (!task_point_b.empty()) {
    steps.push_back({ActionType::MoveTo, task_point_b, {}, {}});
  }

  // 3. task_cartesian_path_reverse 反向 (笛卡尔直线: B → task_pose)
  auto task_cartesian_rev = material_manager_.getTaskCartesianPathReverse();
  if (!task_cartesian_rev.empty()) {
    std::vector<geometry_msgs::msg::Pose> rev_poses;
    for (auto it = task_cartesian_rev.rbegin(); it != task_cartesian_rev.rend(); ++it) {
      rev_poses.push_back(cartesianToPose(*it));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, rev_poses});
  }

  // 4. task_cartesian_path_forward 反向 (笛卡尔直线: task_pose → A)
  auto task_cartesian_fwd = material_manager_.getTaskCartesianPathForward();
  if (!task_cartesian_fwd.empty()) {
    std::vector<geometry_msgs::msg::Pose> rev_poses;
    for (auto it = task_cartesian_fwd.rbegin(); it != task_cartesian_fwd.rend(); ++it) {
      rev_poses.push_back(cartesianToPose(*it));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, rev_poses});
  }

  // 5. task_point_A (关节空间)
  auto task_point_a = material_manager_.getTaskPointA();
  if (!task_point_a.empty()) {
    steps.push_back({ActionType::MoveTo, task_point_a, {}, {}});
  }

  // 6. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  return steps;
}

// ============================================================================
// 物料点部分步骤
// ============================================================================

// Home → na → (笛卡尔) → material_pose → (笛卡尔) → nc → Home
std::vector<Step>
MissionScheduler::buildMaterialForwardSteps(const Material &material) {
  std::vector<Step> steps;

  // 1. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  // 2. forward_points (正向: na, nb, ...)
  for (const auto &wp : material.forward_points) {
    steps.push_back({ActionType::MoveTo, wp, {}, {}});
  }

  // 3. material_cartesian_path_forward (笛卡尔直线: nb → material_pose)
  if (!material.cartesian_path_forward.empty()) {
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto &cp : material.cartesian_path_forward) {
      poses.push_back(cartesianToPose(cp));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, poses});
  }

  // 4. material_cartesian_path_reverse (笛卡尔直线: material_pose → nc)
  if (!material.cartesian_path_reverse.empty()) {
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto &cp : material.cartesian_path_reverse) {
      poses.push_back(cartesianToPose(cp));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, poses});
  }

  // 5. return_point (nc)
  steps.push_back({ActionType::MoveTo, material.return_point.joints, {}, {}});

  // 6. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  return steps;
}

// Home → nc → (笛卡尔) → material_pose → (笛卡尔) → na → Home
std::vector<Step>
MissionScheduler::buildMaterialReturnSteps(const Material &material) {
  std::vector<Step> steps;

  // 1. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  // 2. return_point (nc)
  steps.push_back({ActionType::MoveTo, material.return_point.joints, {}, {}});

  // 3. material_cartesian_path_reverse 反向 (笛卡尔直线: nc → material_pose)
  if (!material.cartesian_path_reverse.empty()) {
    std::vector<geometry_msgs::msg::Pose> rev_poses;
    for (auto it = material.cartesian_path_reverse.rbegin(); it != material.cartesian_path_reverse.rend(); ++it) {
      rev_poses.push_back(cartesianToPose(*it));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, rev_poses});
  }

  // 4. material_cartesian_path_forward 反向 (笛卡尔直线: material_pose → na)
  if (!material.cartesian_path_forward.empty()) {
    std::vector<geometry_msgs::msg::Pose> rev_poses;
    for (auto it = material.cartesian_path_forward.rbegin(); it != material.cartesian_path_forward.rend(); ++it) {
      rev_poses.push_back(cartesianToPose(*it));
    }
    steps.push_back({ActionType::MoveCartesian, {}, {}, rev_poses});
  }

  // 5. forward_points 反向 (na, nb, ...)
  for (auto it = material.forward_points.rbegin(); it != material.forward_points.rend(); ++it) {
    steps.push_back({ActionType::MoveTo, *it, {}, {}});
  }

  // 6. Home
  steps.push_back({ActionType::Home, {}, {}, {}});

  return steps;
}

// ============================================================================
// 步骤执行
// ============================================================================
bool MissionScheduler::executeStep(const Step &step) {
  switch (step.action) {
  case ActionType::Home:
    return executor_->moveHome();
  case ActionType::MoveTo:
    return executor_->moveJoint(step.target);
  case ActionType::MovePath:
    return executor_->moveJointPath(step.waypoints);
  case ActionType::MoveCartesian:
    return executor_->moveCartesianPath(step.cartesian_waypoints);
  }
  return false;
}

} // namespace task_control
