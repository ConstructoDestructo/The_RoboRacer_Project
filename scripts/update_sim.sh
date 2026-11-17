#!/usr/bin/env bash

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${HOME}/sim_ws"
SCRIPT_SOURCE="${REPO_ROOT}/FoxGlove_Simulator_SetUp"
SCRIPT_DESTINATION="${HOME}/sim_ws_setup"
F1TENTH_GYM_SOURCE="${WORKSPACE}/src/f1tenth_gym_ros/f1tenth_gym"

echo "Updating the F1TENTH simulator workspace..."

cd "${REPO_ROOT}"
git pull --ff-only

mkdir -p "${SCRIPT_DESTINATION}"

cp "${SCRIPT_SOURCE}/setup_sim_ws.sh" \
   "${SCRIPT_SOURCE}/setup_sim_env.sh" \
   "${SCRIPT_DESTINATION}/"

chmod +x \
    "${SCRIPT_DESTINATION}/setup_sim_ws.sh" \
    "${SCRIPT_DESTINATION}/setup_sim_env.sh"

bash "${REPO_ROOT}/scripts/link_custom_packages.sh"

SIM_CONFIG_SOURCE="${REPO_ROOT}/simulation/config/sim.yaml"
SIM_CONFIG_DESTINATION="${WORKSPACE}/src/f1tenth_gym_ros/config/sim.yaml"

CUSTOM_MAPS_SOURCE="${REPO_ROOT}/simulation/maps"
CUSTOM_MAPS_DESTINATION="${WORKSPACE}/src/f1tenth_gym_ros/maps"

if [ -f "${SIM_CONFIG_SOURCE}" ]; then
    cp "${SIM_CONFIG_SOURCE}" "${SIM_CONFIG_DESTINATION}"
    echo "Applied repository simulator configuration."
fi

if [ -d "${CUSTOM_MAPS_SOURCE}" ]; then
    cp -a "${CUSTOM_MAPS_SOURCE}/." "${CUSTOM_MAPS_DESTINATION}/"
    echo "Applied repository simulator maps."
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS 2 Humble was not found." >&2
    exit 1
fi

if [ ! -f "${WORKSPACE}/.venv/bin/activate" ]; then
    echo "The simulator virtual environment was not found." >&2
    echo "Run setup_sim_ws.sh first." >&2
    exit 1
fi

if [ ! -d "${F1TENTH_GYM_SOURCE}" ]; then
    echo "f1tenth_gym source directory was not found:" >&2
    echo "${F1TENTH_GYM_SOURCE}" >&2
    exit 1
fi

source "/opt/ros/humble/setup.bash"
source "${WORKSPACE}/.venv/bin/activate"

echo "Installing f1tenth_gym into the simulator virtual environment..."

python3 -m pip install -e "${F1TENTH_GYM_SOURCE}"

if [ ! -x "${WORKSPACE}/.venv/bin/colcon" ]; then
    echo "Installing colcon into the simulator virtual environment..."

    python3 -m pip install \
        --ignore-installed \
        --no-cache-dir \
        colcon-common-extensions
fi

python3 -m pip install \
    --no-cache-dir \
    "empy==3.3.4"

cd "${WORKSPACE}"

apt-get update

rosdep install \
    --ignore-src \
    --from-paths src \
    --rosdistro humble \
    --skip-keys python3-scipy \
    -y

"${WORKSPACE}/.venv/bin/colcon" build --symlink-install

echo
echo "Simulator workspace updated successfully."
echo "Open a new terminal or run:"
echo "source ${WORKSPACE}/install/local_setup.bash"