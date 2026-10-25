#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_SOURCE="$REPO_ROOT/FoxGlove_Simulator_SetUp"
SCRIPT_DESTINATION="$HOME/sim_ws_setup"

echo "Preparing F1TENTH Codespace..."

mkdir -p "$SCRIPT_DESTINATION"
cp "$SCRIPT_SOURCE/setup_sim_ws.sh" \
   "$SCRIPT_SOURCE/setup_sim_env.sh" \
   "$SCRIPT_DESTINATION/"

chmod +x "$SCRIPT_DESTINATION/setup_sim_ws.sh"
chmod +x "$SCRIPT_DESTINATION/setup_sim_env.sh"

# Older versions of this setup added the environment script to .bashrc.
# Remove that block so opening a terminal does not try to source the workspace
# before the first build has completed. launch_sim.sh sources it when needed.
if [ -f "$HOME/.bashrc" ]; then
    sed -i \
        '/^# Automatically prepare the F1TENTH simulator environment\.$/,/^fi$/d' \
        "$HOME/.bashrc"
fi

bash "$REPO_ROOT/scripts/link_custom_packages.sh"

"$SCRIPT_DESTINATION/setup_sim_ws.sh"

echo
echo "Codespace setup complete."
echo "Run ./launch_sim.sh to start the simulator."
