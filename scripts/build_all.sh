#!/usr/bin/env bash

set -euo pipefail

cd "$HOME/star_agibot"

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
  aimdk_msgs \
  race_task \
  voice_asr \
  interaction_agent
