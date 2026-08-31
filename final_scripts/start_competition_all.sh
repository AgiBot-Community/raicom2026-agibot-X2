#!/usr/bin/env bash
set -e

WS="$HOME/star_agibot"
SESSION="race_final"

NAV_PARAMS="$WS/src/race_task/config/competition_official_voice_nav.yaml"

ENV_CMD="cd $WS && \
source /opt/ros/humble/setup.bash && \
source $HOME/aimdk/install/setup.bash && \
source install/setup.bash"

echo "========== Race Final Startup =========="
echo "Workspace: $WS"
echo "Nav params: $NAV_PARAMS"
echo

cd "$WS"

echo "========== 1. Check env =========="
source /opt/ros/humble/setup.bash
source "$HOME/aimdk/install/setup.bash"
source install/setup.bash

python3 - <<'PY'
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource
print("aimdk_msgs import OK")
PY

echo
echo "========== 2. Check nav params =========="
if [ ! -f "$NAV_PARAMS" ]; then
  echo "ERROR: nav params file not found:"
  echo "$NAV_PARAMS"
  echo
  echo "Please create it first and set wait_for_goal_from_rviz: true"
  exit 1
fi

grep -n "wait_for_goal_from_rviz\|goal_pose_topic" "$NAV_PARAMS" || true

if ! grep -q "wait_for_goal_from_rviz: true" "$NAV_PARAMS"; then
  echo
  echo "ERROR: $NAV_PARAMS must use:"
  echo "wait_for_goal_from_rviz: true"
  echo
  echo "Otherwise robot may move immediately after launch."
  exit 1
fi

echo
echo "========== 3. Kill old session/processes =========="
tmux kill-session -t "$SESSION" 2>/dev/null || true

pkill -f fake_localization_pose || true
pkill -f task1_pose_nav || true
pkill -f cmd_vel_bridge || true
pkill -f tf_pose_bridge || true
pkill -f goal_sender || true
pkill -f coordinator_node || true
pkill -f vision_node || true
pkill -f voice_node || true
pkill -f interaction_node || true
pkill -f voice_asr_node || true

sleep 1

echo
echo "========== 4. Start tmux session: $SESSION =========="

tmux new-session -d -s "$SESSION" -n asr \
  "$ENV_CMD && echo '[ASR] starting voice_asr_node' && ros2 run voice_asr voice_asr_node; exec bash"

sleep 1

tmux new-window -t "$SESSION" -n interaction \
  "$ENV_CMD && echo '[INTERACTION] starting interaction_node' && ros2 run interaction_agent interaction_node; exec bash"

sleep 1

tmux new-window -t "$SESSION" -n voice_exec \
  "$ENV_CMD && echo '[VOICE_EXEC] starting voice_node' && ros2 run x2_execution_pkg voice_node; exec bash"

sleep 1

tmux new-window -t "$SESSION" -n vision_exec \
  "$ENV_CMD && echo '[VISION_EXEC] starting vision_node' && ros2 run x2_execution_pkg vision_node; exec bash"

sleep 1

tmux new-window -t "$SESSION" -n nav \
  "$ENV_CMD && echo '[NAV] starting race_task navigation wait mode' && ros2 launch race_task task1_app_ready.launch.py params_file:=$NAV_PARAMS; exec bash"

sleep 2

tmux new-window -t "$SESSION" -n coordinator \
  "$ENV_CMD && echo '[COORDINATOR] starting coordinator_node' && ros2 run x2_execution_pkg coordinator_node; exec bash"

sleep 2

tmux new-window -t "$SESSION" -n listen_on \
  "$ENV_CMD && echo '[LISTEN] enable listen_control after 3 seconds' && sleep 3 && python3 $WS/send_listen_true.py && echo '[LISTEN] listen_control=true sent'; exec bash"

echo
echo "========== Started =========="
echo "tmux session: $SESSION"
echo
echo "查看所有窗口："
echo "  tmux attach -t $SESSION"
echo
echo "查看窗口列表："
echo "  tmux list-windows -t $SESSION"
echo
echo "退出 tmux 但不停止节点："
echo "  Ctrl+B 然后按 D"
echo
echo "停止全部节点："
echo "  tmux kill-session -t $SESSION"
echo
echo "比赛开始后只说语音，不要再操作终端。"
