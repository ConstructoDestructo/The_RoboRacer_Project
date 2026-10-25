# Custom ROS 2 Packages

Place team-developed ROS 2 packages in this directory.

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
