#!/usr/bin/env bash

set -e

source "$HOME/sim_ws_setup/setup_sim_env.sh"

python3 .devcontainer/open_foxglove.py &
REDIRECT_PID=$!

cleanup() {
    kill "$REDIRECT_PID" 2>/dev/null || true
}

trap cleanup EXIT

ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
