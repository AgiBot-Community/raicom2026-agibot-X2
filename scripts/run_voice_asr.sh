#!/usr/bin/env bash

set -euo pipefail

source "$HOME/star_agibot/scripts/env.sh"

ros2 run voice_asr voice_asr_node \
  --ros-args \
  --params-file "$HOME/star_agibot/config/voice_asr.yaml"
