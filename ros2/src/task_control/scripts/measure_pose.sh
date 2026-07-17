#!/bin/bash
# 姿态测量工具
# 用法: 在 RViz 中移动机器人到目标位置后运行此脚本

echo "========================================="
echo "  姿态测量工具"
echo "========================================="
echo ""

# 获取关节角度
echo "[关节角度]"
ros2 topic echo /joint_states --once 2>/dev/null | grep -A1 "position:" | head -2

echo ""

# 获取笛卡尔坐标
echo "[笛卡尔坐标 (base_link → link_6)]"
timeout 2 ros2 run tf2_ros tf2_echo base_link link_6 2>/dev/null | tail -6

echo ""
echo "========================================="
echo "提示: 复制上面的关节角度到 poses.yaml"
echo "========================================="