# The RoboRacer Project

This repository contains the F1TENTH ROS 2 simulator setup and team-developed ROS 2 packages.

## GitHub Codespaces

Use [CODESPACE_GUIDE.md](CODESPACE_GUIDE.md) to create a Codespace, start the simulator, and connect Foxglove.

If you are adding your own package, use [TEAM_PACKAGE_UPLOAD.md](TEAM_PACKAGE_UPLOAD.md) first. Clone the repository with GitHub Desktop, create your personal branch, copy the complete package folder, commit and publish the branch, and only then create the Codespace from that branch.

## Custom packages

Place team packages in [custom_packages](custom_packages/). Each teammate should use a separate package directory and Git branch. The Codespace automatically links this directory into the ROS workspace.

See [custom_packages/README.md](custom_packages/README.md) for the package layout and build commands.

For beginner-friendly instructions on copying a complete package with all of its folders, see [TEAM_PACKAGE_UPLOAD.md](TEAM_PACKAGE_UPLOAD.md).

## Updating an existing Codespace

After new changes are pushed to the repository, run:

```bash
./scripts/update_sim.sh
```

This pulls the current branch, links the custom packages, installs any new ROS dependencies, and rebuilds the workspace.
