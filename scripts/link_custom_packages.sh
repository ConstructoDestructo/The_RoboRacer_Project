#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${HOME}/sim_ws"
SOURCE_DIR="${REPO_ROOT}/custom_packages"
DESTINATION="${WORKSPACE}/src/custom_packages"

mkdir -p "${WORKSPACE}/src"

if [ ! -d "${SOURCE_DIR}" ]; then
    echo "Custom package directory not found: ${SOURCE_DIR}" >&2
    exit 1
fi

# Keep repository packages visible to colcon without copying them into the
# generated workspace. Existing real directories are preserved as backups.
if [ -e "${DESTINATION}" ] && [ ! -L "${DESTINATION}" ]; then
    BACKUP="${DESTINATION}.backup-$(date +%Y%m%d%H%M%S)"
    mv "${DESTINATION}" "${BACKUP}"
    echo "Moved existing custom packages to ${BACKUP}"
fi

ln -sfn "${SOURCE_DIR}" "${DESTINATION}"

echo "Linked custom packages:"
echo "  ${SOURCE_DIR}"
echo "  -> ${DESTINATION}"
