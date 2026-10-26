# F1TENTH Simulator: GitHub Codespaces Guide

This guide explains how to run the F1TENTH simulator in GitHub Codespaces and view it in Foxglove.

Codespaces runs the ROS 2 simulator in the cloud, so teammates do not need a powerful computer or a local ROS 2 installation.

## 1. Prepare your branch and package before creating a Codespace

If you are adding or testing your own ROS 2 package, prepare it before creating the Codespace. This makes sure the Codespace starts with your package already available.

Follow [TEAM_PACKAGE_UPLOAD.md](TEAM_PACKAGE_UPLOAD.md) for the beginner-friendly GitHub Desktop instructions:

1. Install and sign in to GitHub Desktop.
2. Clone `The_RoboRacer_Project`.
3. Switch to the `Shared_CodeBase` branch.
4. Create a personal branch, such as `ricardo-controller` or `marcos-controller`.
5. Copy the complete package folder into:

   ```text
   custom_packages/<your-name>/<package-name>/
   ```

6. Commit the package to your personal branch.
7. Publish the branch to GitHub.

Do not commit personal package work directly to `Shared_CodeBase`. You can create a pull request into `Shared_CodeBase` after testing the package.

## 2. Create the Codespace

1. Open the repository on GitHub.
2. Select the branch you want to use:
   - Use your personal branch if you want to test your own package.
   - Use `Shared_CodeBase` if you only want the shared simulator without unmerged personal work.
3. Select **Code → Codespaces → Create codespace on `<selected branch>`**.
4. Wait for the Codespace container to finish building.

The first setup can take several minutes. It creates the ROS workspace, installs the F1TENTH packages, installs dependencies, and builds the workspace.

The Codespace image includes `nano`, Pillow, SciPy, and the ROS build tools. The setup also installs `colcon` inside the simulator's Python virtual environment so the generated Python ROS executables use the same environment as `f1tenth_gym`.

## 3. Ignore the startup warning

After the Codespace opens, you may see a message similar to:

```text
The workspace has not been built yet.
Run setup_sim_ws.sh first.
```

If the Codespace creation log says **Workspace setup complete** and **Codespace setup complete**, this warning can be ignored. It is caused by the shell checking the environment before the first-time workspace setup has completely finished.

Do not run the workspace setup again unless the setup actually failed or `$HOME/sim_ws` was deleted.

## 4. Launch the simulator

From the repository root, run:

```bash
./launch_sim.sh
```

Keep this terminal open while using the simulator. The command starts the ROS 2 bridge and publishes the simulator data through port `8765`.

## 5. Connect Foxglove

1. Open the **Ports** panel in Codespaces.
2. Find port `8765`, labeled **Foxglove WebSocket**.
3. Copy its **Forwarded Address**.
4. Open [Foxglove](https://app.foxglove.dev).
5. Select **Open connection → Foxglove WebSocket**.
6. Paste the forwarded address into the connection field.
7. Change the address scheme from `https://` to `wss://`.

For example:

```text
https://your-codespace-name-8765.app.github.dev/
```

becomes:

```text
wss://your-codespace-name-8765.app.github.dev/
```

Do not use `ws://localhost:8765`. In a Codespace, `localhost` refers to the local computer rather than the remote simulator.

If Foxglove cannot connect, change port `8765` visibility from **Private** to **Public** or **Organization**, if that option is available. Each Codespace has its own forwarded address, so teammates must copy the address from their own Ports panel.

## 6. Load the visualization layout

After connecting Foxglove, import this layout from the repository:

```text
src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json
```

The layout displays the map, robot models, transforms, and simulator topics.

## 7. Run additional ROS commands

Open a second terminal in the Codespace. Prepare the ROS environment with:

```bash
source "$HOME/sim_ws_setup/setup_sim_env.sh"
```

You can then run other ROS 2 nodes or controllers in that terminal. Leave the terminal running `./launch_sim.sh` open.

## 8. Add and run custom team packages

### What is a package directory?

A package directory is simply a folder containing one ROS 2 program. It keeps the files for one controller together.

For example, a wall-following package might look like this:

```text
custom_packages/ricardo/wall_follow/
├── package.xml       # Describes the ROS package
├── CMakeLists.txt    # Tells ROS 2 how to build the C++ code
└── src/
    └── wall_node.cpp # The actual controller code
```

Each teammate should use their own folder so people do not accidentally edit the same files. For example:

```text
custom_packages/
├── marcos/wall_follow/
├── aaron/safety_controller/
├── brendan/lidar_controller/
└── ricardo/wall_follow/
```

The first folder identifies the teammate. The second folder identifies the ROS package. Do not put multiple people’s source files into one package unless you are intentionally working together on that package.

The package does not need to contain a prebuilt executable. Teammates commit the source code, `package.xml`, and the appropriate build files. The Codespace compiles the executable when `colcon build` runs. Do not commit the generated `build/`, `install/`, or `log/` directories.

### What is a Git branch?

A branch is a separate version of the repository. It lets one person work on changes without immediately changing the shared branch used by everyone else.

Think of it this way:

```text
Shared_CodeBase       # Stable shared version
├── marcos-controller
├── aaron-controller
├── brendan-controller
└── ricardo-controller
```

Each teammate should create their own branch from `Shared_CodeBase`, create their package folder there, and make changes on that branch.

### Beginner workflow

1. On GitHub, open the repository and select the `Shared_CodeBase` branch.
2. Select the branch menu, type your name followed by `-controller`, and create the new branch. For example:

   ```text
   marcos-controller
   ```

3. Create your package inside:

   ```text
   custom_packages/your-name/your-package-name/
   ```

4. Commit your changes to your branch.
5. When your code is ready to share, create a pull request asking to merge your branch into `Shared_CodeBase`.

If someone is only testing code and does not need to share changes yet, they can work in their own branch without creating a pull request.

The Codespace links `custom_packages` into `$HOME/sim_ws/src/custom_packages` during setup. After adding a package, rebuild the workspace:

```bash
cd "$HOME/sim_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

The package source remains in the repository under `custom_packages/`. The generated build files are kept inside the Codespace at:

```text
$HOME/sim_ws/build/    # intermediate compilation files
$HOME/sim_ws/install/  # installed ROS packages and executables
$HOME/sim_ws/log/      # build and test logs
```

These generated directories are not committed to GitHub. Every Codespace creates its own copies when `colcon build` runs.

Run a package from a second terminal:

```bash
ros2 run PACKAGE_NAME EXECUTABLE_NAME
```

The simulator can remain running in the first terminal while each teammate runs a different controller in their own terminal.

To test another teammate's package without changing it, build and run that package directly. To modify it independently, copy the complete package into your own directory and give it a unique package name in `package.xml`.

For beginner-friendly instructions on copying a complete package with all of its folders, see [TEAM_PACKAGE_UPLOAD.md](TEAM_PACKAGE_UPLOAD.md).

## 9. Update an existing Codespace

After new changes are pushed to the current branch, run this from the repository root:

```bash
./scripts/update_sim.sh
```

The update script pulls the branch, links the custom packages, installs new ROS dependencies, and rebuilds the workspace. It uses `git pull --ff-only`, so local uncommitted changes are not overwritten.

## 10. Stop the simulator and Codespace

To stop the simulator, press:

```text
Ctrl + C
```

When finished, stop the Codespace from GitHub so it does not continue consuming Codespaces usage.

## Troubleshooting

### The simulator says that the workspace has not been built

First check whether the setup succeeded. If the creation log contains:

```text
Workspace setup complete.
Codespace setup complete.
```

run:

```bash
./launch_sim.sh
```

If the setup failed, run:

```bash
"$HOME/sim_ws_setup/setup_sim_ws.sh"
```

### Foxglove shows no connection

Confirm all of the following:

- `./launch_sim.sh` is still running.
- Port `8765` appears in the Ports panel.
- The forwarded address uses `wss://` in Foxglove.
- The port visibility allows Foxglove to reach it.
- You are not using `ws://localhost:8765`.

### Keyboard control feels delayed

The simulator is running remotely, so keyboard commands travel through the Codespaces connection. This delay is expected. Codespaces is best for building and testing ROS 2 controllers; local simulation is better for precise manual driving.

### RViz does not open

RViz requires graphical display forwarding and is not the recommended Codespaces workflow. Use Foxglove for visualization.

