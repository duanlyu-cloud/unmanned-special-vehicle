#include "task_control/task_manager.hpp"

#include "robot_executor/robot_executor.hpp"

namespace task_control {

TaskManager::TaskManager(rclcpp::Node::SharedPtr node) : node_(node) {}

bool TaskManager::init(const std::string &yaml_path) {
  if (!material_manager_.loadConfig(yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load poses config: %s",
                 yaml_path.c_str());
    return false;
  }

  executor_ = std::make_shared<robot_executor::RobotExecutor>(node_);
  executor_->init();

  scheduler_ = std::make_unique<MissionScheduler>(node_, executor_,
                                                   material_manager_);

  execute_service_ = node_->create_service<robot_interfaces::srv::ExecuteTask>(
      "/execute_task",
      std::bind(&TaskManager::handleExecuteTask, this,
                std::placeholders::_1, std::placeholders::_2));

  reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/reset_state",
      std::bind(&TaskManager::handleResetState, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(node_->get_logger(), "TaskManager initialized, services ready");
  return true;
}

void TaskManager::handleExecuteTask(
    const std::shared_ptr<robot_interfaces::srv::ExecuteTask::Request> req,
    std::shared_ptr<robot_interfaces::srv::ExecuteTask::Response> res) {

  std::lock_guard<std::mutex> lock(mutex_);

  if (scheduler_->getState() != SchedulerState::Idle) {
    res->accepted = false;
    res->message = "System not idle, reject task: " + req->task.task_id;
    RCLCPP_WARN(node_->get_logger(), "%s", res->message.c_str());
    return;
  }

  res->accepted = true;
  res->message = "Task accepted: " + req->task.task_id;
  RCLCPP_INFO(node_->get_logger(), "Task accepted: type=%d material=%d id=%s",
              req->task.task_type, req->task.material_id,
              req->task.task_id.c_str());

  // Launch task in separate thread so service handler returns immediately
  // This allows /reset_state to be processed during task execution
  if (task_thread_.joinable()) {
    task_thread_.join();
  }

  uint8_t task_type = req->task.task_type;
  uint8_t material_id = req->task.material_id;
  std::string task_id = req->task.task_id;
  task_thread_ = std::thread(&TaskManager::executeTaskAsync, this,
                             task_type, material_id, task_id);
}

void TaskManager::executeTaskAsync(uint8_t task_type, uint8_t material_id,
                                   const std::string &task_id) {
  scheduler_->executeTask(task_type, material_id, task_id);
}

void TaskManager::handleResetState(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  auto state = scheduler_->getState();
  if (state == SchedulerState::Error || state == SchedulerState::Finished) {
    scheduler_->resetState();
    res->success = true;
    res->message = "State reset to Idle";
  } else {
    res->success = false;
    res->message = "Not in Error or Finished state";
  }
}

} // namespace task_control
