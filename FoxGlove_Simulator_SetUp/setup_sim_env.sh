#!/usr/bin/env bash

WORKSPACE="$HOME/sim_ws"

source "$WORKSPACE/.venv/bin/activate"
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
source "$WORKSPACE/install/local_setup.bash"

echo "F1TENTH simulator environment loaded."
echo "Workspace: $WORKSPACE"
echo "Python: $(which python3)"
