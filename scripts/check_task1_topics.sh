#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/star_agibot}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-8}"

echo "========== Check task1 ROS topics =========="
cd "$WORKSPACE"

echo "[1/4] Source ROS Humble"
source /opt/ros/humble/setup.bash

echo "[2/4] Source workspace"
source install/setup.bash

echo "[3/4] Current ROS topics related to task1"
ros2 topic list | grep -E "localization_pose|locomotion|cmd_vel|map_tf" || true

echo
echo "[4/4] Check localization pose:"
echo "Topic: /map_tf_distribution/localization_pose"
if timeout "${TIMEOUT_SECONDS}s" ros2 topic echo /map_tf_distribution/localization_pose --once; then
  echo "✅ localization_pose received."
else
  echo "❌ No localization_pose received within ${TIMEOUT_SECONDS}s."
  echo "可能原因：定位节点没启动、map_tf_distribution 没运行、雷达/定位链路异常。"
fi

echo
echo "Check robot velocity command topic:"
echo "Topic: /aima/mc/locomotion/velocity"
if timeout "${TIMEOUT_SECONDS}s" ros2 topic echo /aima/mc/locomotion/velocity --once; then
  echo "✅ locomotion velocity topic received."
else
  echo "⚠️ No locomotion velocity message received within ${TIMEOUT_SECONDS}s."
  echo "注意：如果当前没有节点发布速度，这是正常的；如果启动导航后仍没有，重点检查 cmd_vel_bridge。"
fi

echo
echo "✅ Topic check finished."
