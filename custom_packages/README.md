# Custom ROS 2 Packages

Place team-developed ROS 2 packages in this directory. A ROS 2 package is a folder containing one ROS program and the files needed to build it.

Recommended layout:

```text
custom_packages/
├── common/
│   └── shared_tools/
├── marcos/
│   └── controller_name/
├── aaron/
│   └── controller_name/
└── teammate_name/
    └── controller_name/
```

See [TEAM_MEMBERS.md](TEAM_MEMBERS.md) for the current team directory list.

## What goes in a package directory?

Each package should have its own folder and normally contains:

```text
your_package/
├── package.xml
├── CMakeLists.txt
└── src/
```

For C++ packages, the source files go inside `src/`. The `package.xml` and `CMakeLists.txt` tell ROS 2 what the package is and how to build it.

Do not place everyone’s source files in the same package. Give each teammate their own package folder unless you are deliberately sharing one package.

## What is a Git branch?

A branch is a separate version of the repository. It allows a teammate to work on code without changing the stable shared branch immediately.

Create a branch from `Shared_CodeBase` using a name such as:

```text
marcos-controller
```

When the code is ready to share, create a pull request to merge the branch back into `Shared_CodeBase`.

Every package directory must contain its own `package.xml` and build file:

- C++ packages: `CMakeLists.txt`
- Python packages: `setup.py` or `setup.cfg`

Each package must have a unique ROS package name in `package.xml`.

The Codespace links this directory into `$HOME/sim_ws/src/custom_packages`, so the packages are built with the simulator without being copied.

To build after adding or changing packages:

```bash
cd "$HOME/sim_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

To build one package only:

```bash
colcon build --packages-select YOUR_PACKAGE_NAME --symlink-install
```
