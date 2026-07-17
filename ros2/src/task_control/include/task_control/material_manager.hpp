#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace task_control {

struct JointPose {
  std::vector<double> joints;
};

struct CartesianPose {
  double x{0.0}, y{0.0}, z{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

struct Material {
  uint8_t id;
  std::string name;
  JointPose material_pose;                     // 物料点关节位姿
  std::vector<std::vector<double>> forward_points;  // 正向中间点 (从远到近)
  JointPose return_point;                      // 返回中间点 (nc)
  std::vector<CartesianPose> cartesian_path_forward;  // nb → material_pose
  std::vector<CartesianPose> cartesian_path_reverse;  // material_pose → nc
};

class MaterialManager {
public:
  bool loadConfig(const std::string &yaml_path);
  std::optional<Material> getMaterial(uint8_t material_id) const;
  JointPose getHomePose() const;
  JointPose getTaskPose() const;
  std::vector<double> getTaskPointA() const;
  std::vector<double> getTaskPointB() const;
  std::vector<CartesianPose> getTaskCartesianPathForward() const;
  std::vector<CartesianPose> getTaskCartesianPathReverse() const;

private:
  JointPose home_pose_;
  JointPose task_pose_;
  std::vector<double> task_point_a_;  // Home → task_pose 中间点
  std::vector<double> task_point_b_;  // task_pose → Home 中间点
  std::vector<CartesianPose> task_cartesian_path_forward_;  // A → task_pose
  std::vector<CartesianPose> task_cartesian_path_reverse_;  // task_pose → B
  std::map<uint8_t, Material> materials_;
};

} // namespace task_control
