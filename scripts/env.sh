#!/usr/bin/env bash

STAR_AGIBOT_ROOT="${STAR_AGIBOT_ROOT:-$HOME/star_agibot}"

source /opt/ros/humble/setup.bash

export PYTHONPATH="$STAR_AGIBOT_ROOT/runtime/python:${PYTHONPATH:-}"

if [ -f "$STAR_AGIBOT_ROOT/install/setup.bash" ]; then
    source "$STAR_AGIBOT_ROOT/install/setup.bash"
fi
