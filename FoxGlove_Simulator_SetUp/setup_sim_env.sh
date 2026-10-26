#!/usr/bin/env bash

WORKSPACE="$HOME/sim_ws"

if [ ! -f "$WORKSPACE/.venv/bin/activate" ]; then
    echo "The Python virtual environment was not found at $WORKSPACE/.venv."
    echo "Run setup_sim_ws.sh first."
    return 1 2>/dev/null || exit 1
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS 2 Humble was not found at /opt/ros/humble."
    return 1 2>/dev/null || exit 1
fi

if [ ! -f "$WORKSPACE/install/local_setup.bash" ]; then
    echo "The workspace has not been built yet."
    echo "Run setup_sim_ws.sh first."
    return 1 2>/dev/null || exit 1
fi

source "$WORKSPACE/.venv/bin/activate"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/local_setup.bash"

# Keep the venv interpreter first even after sourcing ROS and the workspace.
export PATH="$WORKSPACE/.venv/bin:$PATH"

echo "F1TENTH simulator environment loaded."
echo "Workspace: $WORKSPACE"
echo "Python: $(which python3)"
