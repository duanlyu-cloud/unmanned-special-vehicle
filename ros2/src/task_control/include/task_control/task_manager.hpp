#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/execute_task.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "task_control/material_manager.hpp"
#include "task_control/mission_scheduler.hpp"

namespace task_control {

class TaskManager {
public:
  TaskManager(rclcpp::Node::SharedPtr node);

  bool init(const std::string &yaml_path);

  bool moveHome();

private:
  void handleExecuteTask(
      const std::shared_ptr<robot_interfaces::srv::ExecuteTask::Request> req,
      std::shared_ptr<robot_interfaces::srv::ExecuteTask::Response> res);

  void handleResetState(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void executeTaskAsync(uint8_t task_type, uint8_t material_id,
                        const std::string &task_id);

  rclcpp::Node::SharedPtr node_;
  MaterialManager material_manager_;
  std::shared_ptr<robot_executor::RobotExecutor> executor_;
  std::unique_ptr<MissionScheduler> scheduler_;

  rclcpp::Service<robot_interfaces::srv::ExecuteTask>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  std::mutex mutex_;
  std::thread task_thread_;
  rclcpp::TimerBase::SharedPtr home_timer_;
};

} // namespace task_control
