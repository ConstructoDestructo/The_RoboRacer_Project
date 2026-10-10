#!/usr/bin/env bash

set -e

source "$HOME/sim_ws_setup/setup_sim_env.sh"

PORT=8765
FORWARDED_HOST="${CODESPACE_NAME}-${PORT}.${GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN}"
FOXGLOVE_URL="https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=wss://${FORWARDED_HOST}"

echo
echo "Simulator is starting..."
echo
echo "Open this Foxglove link in your browser:"
echo "$FOXGLOVE_URL"
echo
echo "If Foxglove cannot connect, set port $PORT visibility to Public."
echo

exec ros2 launch f1tenth_gym_ros gym_bridge_launch.py open_foxglove:=false
