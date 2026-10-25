#!/usr/bin/env bash

set -e

source "$HOME/sim_ws_setup/setup_sim_env.sh"

exec ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
