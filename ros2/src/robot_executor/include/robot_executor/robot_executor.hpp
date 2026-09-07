#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robot_executor {

class RobotExecutor {
public:
  explicit RobotExecutor(rclcpp::Node::SharedPtr node);

  bool init();

  bool moveHome();
  bool moveJoint(const std::vector<double> &joints);
  bool moveJointPath(const std::vector<std::vector<double>> &waypoints);
  bool moveCartesianPath(const std::vector<geometry_msgs::msg::Pose> &waypoints);
  bool movePose(const geometry_msgs::msg::Pose &pose);

  void stop();
  bool isMoving() const;
  void enableClosedLoop();
  void disableClosedLoop();

private:
  bool executeWithTimeout(std::function<bool()> action,
                          const std::string &action_name);
  std::vector<double> getCurrentJointValues();

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool waitForFreshJointState(std::chrono::milliseconds max_wait);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::chrono::milliseconds timeout_{60000};

  std::atomic<bool> moving_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<double> last_joint_positions_;
  std::mutex joint_state_mutex_;
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
};

} // namespace robot_executor
