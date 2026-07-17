#include "task_control/material_manager.hpp"

#include <rclcpp/rclcpp.hpp>

namespace task_control {

bool MaterialManager::loadConfig(const std::string &yaml_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    // 加载 home_pose
    auto home_joints = config["home_pose"]["joints"];
    for (const auto &j : home_joints) {
      home_pose_.joints.push_back(j.as<double>());
    }
    if (home_pose_.joints.size() != 6) {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "home_pose must have exactly 6 joints, got %zu",
                   home_pose_.joints.size());
      return false;
    }

    // 加载 task_point_A
    if (config["task_point_A"]) {
      for (const auto &j : config["task_point_A"]) {
        task_point_a_.push_back(j.as<double>());
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "Missing required field: task_point_A");
      return false;
    }

    // 加载 task_point_B
    if (config["task_point_B"]) {
      for (const auto &j : config["task_point_B"]) {
        task_point_b_.push_back(j.as<double>());
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "Missing required field: task_point_B");
      return false;
    }

    // 加载 task_pose
    auto task_joints = config["task_pose"]["joints"];
    for (const auto &j : task_joints) {
      task_pose_.joints.push_back(j.as<double>());
    }
    if (task_pose_.joints.size() != 6) {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "task_pose must have exactly 6 joints, got %zu",
                   task_pose_.joints.size());
      return false;
    }

    // 加载 task_cartesian_path_forward
    if (config["task_cartesian_path_forward"]) {
      for (const auto &wp : config["task_cartesian_path_forward"]) {
        CartesianPose cp;
        cp.x = wp["x"].as<double>();
        cp.y = wp["y"].as<double>();
        cp.z = wp["z"].as<double>();
        cp.qx = wp["qx"].as<double>();
        cp.qy = wp["qy"].as<double>();
        cp.qz = wp["qz"].as<double>();
        cp.qw = wp["qw"].as<double>();
        task_cartesian_path_forward_.push_back(cp);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "Missing required field: task_cartesian_path_forward");
      return false;
    }

    // 加载 task_cartesian_path_reverse
    if (config["task_cartesian_path_reverse"]) {
      for (const auto &wp : config["task_cartesian_path_reverse"]) {
        CartesianPose cp;
        cp.x = wp["x"].as<double>();
        cp.y = wp["y"].as<double>();
        cp.z = wp["z"].as<double>();
        cp.qx = wp["qx"].as<double>();
        cp.qy = wp["qy"].as<double>();
        cp.qz = wp["qz"].as<double>();
        cp.qw = wp["qw"].as<double>();
        task_cartesian_path_reverse_.push_back(cp);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                   "Missing required field: task_cartesian_path_reverse");
      return false;
    }

    // 加载 materials
    auto materials = config["materials"];
    for (const auto &item : materials) {
      Material mat;

      // 从 YAML key 提取 ID (例如 "material1" → 1, "material4" → 4)
      std::string key = item.first.as<std::string>();
      uint8_t id = static_cast<uint8_t>(std::stoi(key.substr(8)));  // "material" 长度为8
      mat.id = id;
      mat.name = item.second["name"].as<std::string>();

      // 加载 material_pose
      for (const auto &j : item.second["material_pose"]) {
        mat.material_pose.joints.push_back(j.as<double>());
      }
      if (mat.material_pose.joints.size() != 6) {
        RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                     "Material '%s': material_pose must have exactly 6 joints, got %zu",
                     mat.name.c_str(), mat.material_pose.joints.size());
        return false;
      }

      // 加载 forward_points (可变长度列表)
      if (item.second["forward_points"]) {
        for (const auto &wp : item.second["forward_points"]) {
          std::vector<double> waypoint;
          for (const auto &j : wp) {
            waypoint.push_back(j.as<double>());
          }
          mat.forward_points.push_back(waypoint);
        }
      }
      if (mat.forward_points.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                     "Material '%s': forward_points cannot be empty",
                     mat.name.c_str());
        return false;
      }

      // 加载 return_point
      if (item.second["return_point"]) {
        for (const auto &j : item.second["return_point"]) {
          mat.return_point.joints.push_back(j.as<double>());
        }
      }
      if (mat.return_point.joints.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                     "Material '%s': return_point cannot be empty",
                     mat.name.c_str());
        return false;
      }

      // 加载 material_cartesian_path_forward (nb → material_pose)
      if (item.second["material_cartesian_path_forward"]) {
        for (const auto &wp : item.second["material_cartesian_path_forward"]) {
          CartesianPose cp;
          cp.x = wp["x"].as<double>();
          cp.y = wp["y"].as<double>();
          cp.z = wp["z"].as<double>();
          cp.qx = wp["qx"].as<double>();
          cp.qy = wp["qy"].as<double>();
          cp.qz = wp["qz"].as<double>();
          cp.qw = wp["qw"].as<double>();
          mat.cartesian_path_forward.push_back(cp);
        }
      }

      // 加载 material_cartesian_path_reverse (material_pose → nc)
      if (item.second["material_cartesian_path_reverse"]) {
        for (const auto &wp : item.second["material_cartesian_path_reverse"]) {
          CartesianPose cp;
          cp.x = wp["x"].as<double>();
          cp.y = wp["y"].as<double>();
          cp.z = wp["z"].as<double>();
          cp.qx = wp["qx"].as<double>();
          cp.qy = wp["qy"].as<double>();
          cp.qz = wp["qz"].as<double>();
          cp.qw = wp["qw"].as<double>();
          mat.cartesian_path_reverse.push_back(cp);
        }
      }

      materials_[id] = mat;
    }

    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MaterialManager"),
                 "YAML parse error in %s: %s", yaml_path.c_str(), e.what());
    return false;
  }
}

std::optional<Material>
MaterialManager::getMaterial(uint8_t material_id) const {
  auto it = materials_.find(material_id);
  if (it != materials_.end()) {
    return it->second;
  }
  return std::nullopt;
}

JointPose MaterialManager::getHomePose() const { return home_pose_; }

JointPose MaterialManager::getTaskPose() const { return task_pose_; }

std::vector<double> MaterialManager::getTaskPointA() const {
  return task_point_a_;
}

std::vector<double> MaterialManager::getTaskPointB() const {
  return task_point_b_;
}

std::vector<CartesianPose> MaterialManager::getTaskCartesianPathForward() const {
  return task_cartesian_path_forward_;
}

std::vector<CartesianPose> MaterialManager::getTaskCartesianPathReverse() const {
  return task_cartesian_path_reverse_;
}

} // namespace task_control
