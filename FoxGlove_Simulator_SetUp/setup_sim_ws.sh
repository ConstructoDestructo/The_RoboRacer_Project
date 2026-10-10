#!/usr/bin/env bash

set -e

WORKSPACE="$HOME/sim_ws"
SRC_DIR="$WORKSPACE/src"
ROS_SETUP="/opt/ros/humble/setup.bash"

echo "Setting up F1TENTH ROS 2 workspace..."

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS 2 Humble is not installed."
    echo "Install ROS 2 Humble first:"
    echo "https://docs.ros.org/en/humble/Installation.html"
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
python3 -m pip install scipy

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

source "$ROS_SETUP"

cd "$WORKSPACE"

rosdep install \
    --ignore-src \
    --from-paths src \
    --rosdistro humble \
    --skip-keys python3-scipy \
    -y

colcon build

echo
echo "Workspace setup complete."
echo "Use the following command to activate the simulator environment:"
echo "source $HOME/sim_ws_setup/setup_sim_env.sh"
