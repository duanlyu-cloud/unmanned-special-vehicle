#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include "robot_executor/robot_executor.hpp"
#include "robot_interfaces/msg/task_result.hpp"
#include "robot_interfaces/srv/chassis_move.hpp"
#include "task_control/material_manager.hpp"

namespace task_control {

enum class ActionType { Home, MoveTo, MovePath, MoveCartesian, ChassisMove };

struct Step {
  ActionType action;
  std::vector<double> target;
  std::vector<std::vector<double>> waypoints;
  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
  std::string chassis_station;  // Target station for ChassisMove
};

enum class SchedulerState { Idle, Running, Finished, Error };

class MissionScheduler {
public:
  MissionScheduler(rclcpp::Node::SharedPtr node,
                   std::shared_ptr<robot_executor::RobotExecutor> executor,
                   const MaterialManager &material_manager);

  void executeTask(uint8_t task_type, uint8_t material_id,
                   const std::string &task_id);

  SchedulerState getState() const;
  void resetState();
  std::string getLastResult() const;

private:
  // Task 1: 放任务 (取任务点物料 → 放回物料点)
  std::vector<Step> buildTask1Steps(const Material &material);
  // Task 2: 取任务 (取物料点物料 → 放到任务点)
  std::vector<Step> buildTask2Steps(const Material &material);

  // 任务点部分步骤 (Task1 和 Task2 共用，只是调用顺序不同)
  std::vector<Step> buildTaskPointForwardSteps();  // Home → A → task_pose
  std::vector<Step> buildTaskPointReturnSteps();   // task_pose → B → Home

  // 物料点部分步骤
  std::vector<Step> buildMaterialForwardSteps(const Material &material);
  std::vector<Step> buildMaterialReturnSteps(const Material &material);

  bool executeStep(const Step &step);
  void publishResult(const std::string &task_id, uint8_t result_code,
                     const std::string &message);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<robot_executor::RobotExecutor> executor_;
  const MaterialManager &material_manager_;

  rclcpp::Publisher<robot_interfaces::msg::TaskResult>::SharedPtr result_pub_;
  rclcpp::Client<robot_interfaces::srv::ChassisMove>::SharedPtr chassis_client_;

  bool moveChassis(const std::string &station_id);

  SchedulerState state_{SchedulerState::Idle};
  std::string last_result_;
};

} // namespace task_control
