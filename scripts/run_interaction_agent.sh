#!/usr/bin/env bash

set -euo pipefail

source "$HOME/star_agibot/scripts/env.sh"

ros2 run interaction_agent interaction_node
