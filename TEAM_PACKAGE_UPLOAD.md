# Uploading a ROS 2 Package as a Team Member

The GitHub website is not the easiest way to upload a ROS 2 package because packages often contain several nested folders. Use GitHub Desktop instead. It lets you copy the complete package folder normally, preserving its structure, and then commit the complete change.

## 1. Install and sign in to GitHub Desktop

Install [GitHub Desktop](https://desktop.github.com/) and sign in with your GitHub account.

## 2. Clone the repository

In GitHub Desktop:

1. Select **File → Clone repository**.
2. Choose the **URL** tab.
3. Enter:

   ```text
   https://github.com/ConstructoDestructo/The_RoboRacer_Project.git
   ```

4. Choose where to store the repository on your computer.
5. Select **Clone**.

## 3. Create your own branch

In the **Current Branch** menu:

1. Select `Shared_CodeBase`.
2. Select **New Branch**.
3. Name the branch using your name and the type of work. For example:

   ```text
   marcos-controller
   ```

4. Make sure the new branch is based on `Shared_CodeBase`.
5. Select **Create Branch**.

## 4. Copy the complete package folder

In GitHub Desktop, select **Repository → Show in Explorer**.

Open this folder in File Explorer:

```text
custom_packages/<your-name>/
```

Copy your entire ROS package folder into that location. For example, copy the complete `wall_follow` folder into:

```text
custom_packages/marcos/wall_follow/
```

The result should look like:

```text
custom_packages/
└── marcos/
    └── wall_follow/
        ├── package.xml
        ├── CMakeLists.txt
        └── src/
            └── wall_node.cpp
```

If the package contains `include/`, `launch/`, `config/`, or `test/` directories, copy those too. Copy the entire package folder rather than uploading its files one at a time.

Do not copy generated folders such as `build/`, `install/`, or `log/`.

## 5. Commit and publish the branch

Return to GitHub Desktop. It should show the package files as changed files.

1. Enter a summary such as:

   ```text
   Add Marcos wall-following package
   ```

2. Select **Commit to marcos-controller**.
3. Select **Publish branch** or **Push origin**.

## 6. Create a Codespace from your branch

Now that your package has been committed and published, create the Codespace from your personal branch—not from `Shared_CodeBase`.

1. Open the repository on GitHub.
2. Select your personal branch, such as `marcos-controller`.
3. Select **Code → Codespaces → Create codespace on `<your branch>`**.
4. Wait for the Codespace setup to finish.

After the Codespace opens, update and build the workspace:

```bash
./scripts/update_sim.sh
```

Then build and run the package:

```bash
cd "$HOME/sim_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-select PACKAGE_NAME --symlink-install
source install/local_setup.bash
ros2 run PACKAGE_NAME EXECUTABLE_NAME
```

Keep the simulator running in another terminal with:

```bash
./launch_sim.sh
```

## 7. Share the package with the team

When the package is ready to share, create a pull request from your personal branch into `Shared_CodeBase`. Do not delete your personal branch until the pull request has been reviewed and merged.
