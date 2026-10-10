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

if ! grep -Fq 'source "$HOME/sim_ws_setup/setup_sim_env.sh"' "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" <<'BASHRC'

# Automatically prepare the F1TENTH simulator environment.
if [ -f "$HOME/sim_ws_setup/setup_sim_env.sh" ]; then
    source "$HOME/sim_ws_setup/setup_sim_env.sh"
fi
BASHRC
fi

"$SCRIPT_DESTINATION/setup_sim_ws.sh"

echo
echo "Codespace setup complete."
echo "Run ./launch_sim.sh to start the simulator."
