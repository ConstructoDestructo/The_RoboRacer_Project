# F1TENTH Simulator Environment Setup

This guide explains how to install and automatically prepare the F1TENTH ROS 2 Humble simulation environment.

## Recommended: GitHub Codespaces

This repository includes a Codespaces configuration that runs the simulator in a cloud development environment. This is useful when a teammate's computer does not have enough CPU, memory, or graphics capability to run the simulator locally.

### Create a Codespace

1. Open this repository on GitHub.
2. Select the `Ricky's-CodeSpace` branch.
3. Select **Code → Codespaces → Create codespace on Ricky's-CodeSpace**.
4. Wait for Codespaces to build the development container.

The container installs ROS 2 Humble and the required build tools. Its first-time setup then creates `$HOME/sim_ws`, clones the F1TENTH repositories, installs `f1tenth_gym`, installs ROS dependencies, and builds the workspace.

The setup also builds `ackermann_msgs` from its ROS 2 source branch because the binary apt package is not available in every ROS Humble container configuration.

After the Codespace finishes creating, start the simulator with:

```bash
./launch_sim.sh
```

The launcher uses `open_foxglove:=false` because the Codespace cannot open a browser on the remote container. Port `8765` is forwarded automatically for the Foxglove WebSocket connection.

### Connect Foxglove

1. Leave the simulator terminal running.
2. Open the **PORTS** panel in the Codespace.
3. Find port `8765` labeled **Foxglove WebSocket**.
4. Open Foxglove in a browser.
5. Select **Open connection → Foxglove WebSocket**.
6. Use the forwarded URL for port `8765` rather than `ws://localhost:8765`.

The local `ws://localhost:8765` address only works when Foxglove and the simulator are running on the same computer. In a Codespace, the forwarded port URL is required.

If Codespaces displays the forwarded address as an `https://` URL, use the same host with the WebSocket scheme `wss://` when Foxglove asks for a Foxglove WebSocket connection.

Import this layout into Foxglove for the recommended visualization:

```text
sim_ws/src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json
```

RViz is not the default Codespaces workflow because it requires a remote graphical display. Foxglove is recommended for Codespaces.

### Codespaces resource and cost notes

Codespaces run on cloud virtual machines. Machine sizes, availability, and billing depend on the GitHub account or organization. Stop the Codespace when it is not being used so it does not continue consuming resources.

The simulator still sends visualization data to the local browser, so Foxglove uses some local graphics resources even though the ROS simulation runs remotely.

## Prerequisites

Install the following before running the setup script:

- Ubuntu with Bash
- ROS 2 Humble
- Git
- Python 3
- Python virtual-environment support
- `rosdep`
- `colcon`

If Python virtual-environment support is missing, install it with:

```bash
sudo apt update
sudo apt install python3-venv
```

Follow the official ROS 2 Humble installation instructions before continuing:

<https://docs.ros.org/en/humble/Installation.html>

## 1. Get the setup scripts

Clone the setup repository and check out the Codespaces branch:

```bash
git clone --branch "Ricky's-CodeSpace" \
    https://github.com/ConstructoDestructo/The_RoboRacer_Project.git \
    "$HOME/f1tenth-sim-setup"
```

Create the directory that will store the setup scripts:

```bash
mkdir -p "$HOME/sim_ws_setup"
```

Copy the scripts into that directory:

```bash
cp "$HOME/f1tenth-sim-setup/FoxGlove_Simulator_SetUp/setup_sim_ws.sh" \
   "$HOME/f1tenth-sim-setup/FoxGlove_Simulator_SetUp/setup_sim_env.sh" \
   "$HOME/sim_ws_setup/"
```

Make the scripts executable:

```bash
chmod +x "$HOME/sim_ws_setup/setup_sim_ws.sh"
chmod +x "$HOME/sim_ws_setup/setup_sim_env.sh"
```

The expected directory layout is:

```text
$HOME/
├── f1tenth-sim-setup/
│   └── FoxGlove_Simulator_SetUp/
│       ├── setup_sim_ws.sh
│       ├── setup_sim_env.sh
│       └── SIMULATOR_SETUP.md
├── sim_ws_setup/
│   ├── setup_sim_ws.sh
│   └── setup_sim_env.sh
└── sim_ws/
    ├── .venv/
    ├── install/
    ├── log/
    └── src/
```

The `sim_ws` directory is created by the workspace setup script. The `sim_ws_setup` directory must be created manually because it stores the scripts separately from the ROS workspace.

## 2. Create and build the ROS workspace

Run the workspace setup script once:

```bash
"$HOME/sim_ws_setup/setup_sim_ws.sh"
```

The script will:

1. Create `$HOME/sim_ws/src`.
2. Create a Python virtual environment at `$HOME/sim_ws/.venv`.
3. Clone the `f1tenth_gym_ros` repository using the `dev-humble` branch.
4. Clone `f1tenth_gym` if it is not already present.
5. Install `f1tenth_gym` with pip.
6. Install ROS dependencies with `rosdep`.
7. Build the workspace with `colcon`.

This step may take several minutes. It normally only needs to be repeated when the workspace is deleted or the source code changes significantly.

## 3. Prepare new terminal sessions

For native Ubuntu installations, you may optionally source the simulator environment automatically in new Bash sessions. This is not recommended in Codespaces because the shell may try to source the workspace before its first build has completed.

In Codespaces, use `./launch_sim.sh` for the simulator terminal and source the environment manually in additional terminals:

```bash
source "$HOME/sim_ws_setup/setup_sim_env.sh"
```

For a native Ubuntu installation, continue with the optional `.bashrc` setup below.

Open the Bash startup configuration file:

```bash
nano "$HOME/.bashrc"
```

Add the following block at the bottom:

```bash
if [ -f "$HOME/sim_ws_setup/setup_sim_env.sh" ]; then
    source "$HOME/sim_ws_setup/setup_sim_env.sh"
fi
```

Save and exit Nano:

1. Press `Ctrl + O`.
2. Press `Enter`.
3. Press `Ctrl + X`.

Apply the change immediately:

```bash
source "$HOME/.bashrc"
```

Every new interactive Bash terminal will now automatically:

- Activate the Python virtual environment.
- Source the ROS 2 Humble environment.
- Move into `$HOME/sim_ws`.
- Source the local workspace environment.

## 4. Verify the environment

Run:

```bash
echo "$ROS_DISTRO"
which python3
pwd
```

Expected output should include:

```text
humble
/home/<username>/sim_ws/.venv/bin/python3
/home/<username>/sim_ws
```

The setup script should also print:

```text
F1TENTH simulator environment loaded.
```

## Manual activation

If automatic activation is not enabled, prepare the environment manually with:

```bash
source "$HOME/sim_ws_setup/setup_sim_env.sh"
```

## Team packages

In the repository, place custom ROS 2 packages under:

```text
custom_packages/<team-member>/<package-name>/
```

Codespaces automatically links this directory into `$HOME/sim_ws/src/custom_packages`. After adding or changing a package, rebuild with:

```bash
cd "$HOME/sim_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

To update an existing Codespace from the current Git branch, run this from the repository root:

```bash
./scripts/update_sim.sh
```

## Troubleshooting

### `setup_sim_env.sh: No such file or directory`

Confirm that the scripts are in the expected directory:

```bash
ls -l "$HOME/sim_ws_setup"
```

### `install/local_setup.bash: No such file or directory`

The workspace has not been built yet. Run:

```bash
"$HOME/sim_ws_setup/setup_sim_ws.sh"
```

### The script is not executable

Run:

```bash
chmod +x "$HOME/sim_ws_setup/setup_sim_ws.sh"
chmod +x "$HOME/sim_ws_setup/setup_sim_env.sh"
```

### ROS 2 Humble cannot be found

Verify that the ROS setup file exists:

```bash
ls /opt/ros/humble/setup.bash
```

If it does not exist, install ROS 2 Humble before running the workspace setup script.
