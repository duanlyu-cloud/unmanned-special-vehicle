#include "robot_executor/robot_executor.hpp"

#include <future>

namespace robot_executor {

RobotExecutor::RobotExecutor(rclcpp::Node::SharedPtr node) : node_(node) {}

bool RobotExecutor::init() {
  move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_, "ar_manipulator");

  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(3);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local();
  qos.reliable();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos,
      std::bind(&RobotExecutor::jointStateCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "RobotExecutor initialized");
  return true;
}

void RobotExecutor::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  last_joint_positions_ = msg->position;
  last_joint_state_time_ = msg->header.stamp;
}

bool RobotExecutor::waitForFreshJointState(
    std::chrono::milliseconds max_wait) {
  auto start = std::chrono::steady_clock::now();
  rclcpp::Time last_time(0, 0, RCL_ROS_TIME);

  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    last_time = last_joint_state_time_;
  }

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (last_joint_state_time_ > last_time) {
      return true;
    }

    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= max_wait) {
      RCLCPP_WARN(node_->get_logger(),
                  "waitForFreshJointState: timeout after %ld ms",
                  max_wait.count());
      return false;
    }
  }
  return false;
}

std::vector<double> RobotExecutor::getCurrentJointValues() {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (last_joint_positions_.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "No joint state received yet, using zeros");
    return std::vector<double>(6, 0.0);
  }
  return last_joint_positions_;
}

bool RobotExecutor::moveHome() {
  return executeWithTimeout(
      [this]() {
        move_group_->setNamedTarget("home");
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        auto result = move_group_->move();
        return (result == moveit::core::MoveItErrorCode::SUCCESS);
      },
      "moveHome");
}

bool RobotExecutor::moveJoint(const std::vector<double> &joints) {
  if (joints.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "moveJoint: empty joint vector");
    return false;
  }
  return executeWithTimeout(
      [this, joints]() {
        move_group_->setJointValueTarget(joints);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        auto result = move_group_->move();
        return (result == moveit::core::MoveItErrorCode::SUCCESS);
      },
      "moveJoint");
}

bool RobotExecutor::moveJointPath(
    const std::vector<std::vector<double>> &waypoints) {
  for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(), "moveJointPath: waypoint %zu/%zu", i + 1,
                waypoints.size());
    if (!moveJoint(waypoints[i])) {
      RCLCPP_ERROR(node_->get_logger(), "moveJointPath: failed at waypoint %zu",
                   i + 1);
      return false;
    }
  }
  return true;
}

bool RobotExecutor::moveCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> &waypoints) {
  return executeWithTimeout(
      [this, waypoints]() {
        move_group_->setEndEffectorLink("link_6");
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        if (!waitForFreshJointState(std::chrono::milliseconds(500))) {
          RCLCPP_WARN(node_->get_logger(),
                      "Using stale joint state for Cartesian path");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Computing Cartesian path with %zu waypoints", waypoints.size());
        for (size_t i = 0; i < waypoints.size(); ++i) {
          RCLCPP_INFO(node_->get_logger(),
                      "  wp%zu: [%.3f, %.3f, %.3f]", i,
                      waypoints[i].position.x, waypoints[i].position.y,
                      waypoints[i].position.z);
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = 0.0;
        const int max_retries = 3;

        for (int attempt = 0; attempt < max_retries; ++attempt) {
          move_group_->setStartStateToCurrentState();
          trajectory = moveit_msgs::msg::RobotTrajectory();
          fraction = move_group_->computeCartesianPath(
              waypoints, 0.02, 0.0, trajectory);

          RCLCPP_INFO(node_->get_logger(),
                      "Cartesian path: %.1f%% complete (attempt %d/%d)",
                      fraction * 100.0, attempt + 1, max_retries);

          if (fraction >= 0.90) {
            break;
          }

          if (attempt < max_retries - 1) {
            RCLCPP_WARN(node_->get_logger(),
                        "Cartesian path too incomplete (%.1f%%), retrying...",
                        fraction * 100.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }

        if (fraction < 0.90) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Cartesian path failed after %d attempts (%.1f%%)",
                       max_retries, fraction * 100.0);
          return false;
        }

        return (move_group_->execute(trajectory) ==
                moveit::core::MoveItErrorCode::SUCCESS);
      },
      "moveCartesianPath");
}

bool RobotExecutor::movePose(const geometry_msgs::msg::Pose &pose) {
  return executeWithTimeout(
      [this, pose]() {
        move_group_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) !=
            moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(), "Planning failed for movePose");
          return false;
        }
        return (move_group_->execute(plan) ==
                moveit::core::MoveItErrorCode::SUCCESS);
      },
      "movePose");
}

void RobotExecutor::stop() {
  move_group_->stop();
  RCLCPP_WARN(node_->get_logger(), "RobotExecutor: motion stopped");
}

bool RobotExecutor::isMoving() const { return moving_; }

bool RobotExecutor::executeWithTimeout(std::function<bool()> action,
                                       const std::string &action_name) {
  moving_ = true;
  auto future = std::async(std::launch::async, action);

  if (future.wait_for(timeout_) == std::future_status::timeout) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] Timeout after %ld ms, stopping motion",
                 action_name.c_str(), timeout_.count());
    stop();
    moving_ = false;
    // Wait for async task to finish to prevent future destructor from calling std::terminate
    future.wait();
    return false;
  }

  bool result = false;
  try {
    result = future.get();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Exception: %s", action_name.c_str(),
                 e.what());
    moving_ = false;
    return false;
  }

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Execution failed",
                 action_name.c_str());
  }
  moving_ = false;
  return result;
}

} // namespace robot_executor
