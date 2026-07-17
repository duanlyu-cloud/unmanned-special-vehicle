#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "task_control/task_manager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_manager");

  auto task_manager = std::make_shared<task_control::TaskManager>(node);

  std::string yaml_path;
  node->declare_parameter("poses_yaml_path", "");
  node->get_parameter("poses_yaml_path", yaml_path);

  if (yaml_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "poses_yaml_path parameter not set");
    return 1;
  }

  if (!task_manager->init(yaml_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize TaskManager");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "TaskManager node running");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
