#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="$HOME/sim_ws"
SRC_DIR="$WORKSPACE/src"
ROS_SETUP="/opt/ros/humble/setup.bash"

echo "Setting up F1TENTH ROS 2 workspace..."

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS 2 Humble is not installed."
    exit 1
fi

mkdir -p "$SRC_DIR"

ACKERMANN_DIR="$SRC_DIR/ackermann_msgs"

if [ ! -d "$ACKERMANN_DIR" ]; then
    git clone -b ros2 \
        https://github.com/ros-drivers/ackermann_msgs.git \
        "$ACKERMANN_DIR"
else
    echo "ackermann_msgs already exists. Skipping clone."
fi

if [ ! -d "$WORKSPACE/.venv" ]; then
    python3 -m venv --system-site-packages "$WORKSPACE/.venv"
fi

source "$WORKSPACE/.venv/bin/activate"

python3 -m pip install --upgrade pip

# Install colcon inside the virtual environment.
python3 -m pip install \
    --ignore-installed \
    --no-cache-dir \
    colcon-common-extensions

# ROS 2 Humble requires this older EmPy version.
python3 -m pip install \
    --force-reinstall \
    --no-cache-dir \
    "empy==3.3.4"

cd "$SRC_DIR"

if [ ! -d "$SRC_DIR/f1tenth_gym_ros" ]; then
    git clone -b dev-humble \
        https://github.com/f1tenth/f1tenth_gym_ros.git
else
    echo "f1tenth_gym_ros already exists. Skipping clone."
fi

GYM_ROS_DIR="$SRC_DIR/f1tenth_gym_ros"
GYM_DIR="$GYM_ROS_DIR/f1tenth_gym"

cd "$GYM_ROS_DIR"

if [ ! -d "$GYM_DIR" ]; then
    git clone -b dev-humble \
        https://github.com/f1tenth/f1tenth_gym.git
else
    echo "f1tenth_gym already exists. Skipping clone."
fi

cd "$GYM_DIR"

python3 -m pip install -e .

# ROS 2 Humble references this variable while loading its setup script.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

source "$ROS_SETUP"

cd "$WORKSPACE"

apt-get update

rosdep install \
    --ignore-src \
    --from-paths src \
    --rosdistro humble \
    --skip-keys python3-scipy \
    -y

hash -r

"$WORKSPACE/.venv/bin/colcon" build --symlink-install

echo
echo "Workspace setup complete."
echo
echo "Activate the simulator environment with:"
echo
echo "source \$HOME/sim_ws_setup/setup_sim_env.sh"
