# F1TENTH Simulator Environment Setup

This guide explains how to install and automatically prepare the F1TENTH ROS 2 Humble simulation environment.

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

Clone this repository, replacing `<repository-url>` with the URL for this GitHub repository:

```bash
git clone <repository-url> "$HOME/f1tenth-sim-setup"
```

Create the directory that will store the setup scripts:

```bash
mkdir -p "$HOME/sim_ws_setup"
```

Copy the scripts into that directory:

```bash
cp "$HOME/f1tenth-sim-setup/setup_sim_ws.sh" \
   "$HOME/f1tenth-sim-setup/setup_sim_env.sh" \
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
│   ├── setup_sim_ws.sh
│   └── setup_sim_env.sh
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

## 3. Automatically prepare new terminal sessions

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
