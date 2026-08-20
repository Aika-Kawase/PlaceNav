#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
CAMERA_CONFIG="${SCRIPT_DIR}/config/camera.yaml"
CAMERA_CALIBRATION="${SCRIPT_DIR}/config/calibration.yaml"
TOPOMAP_BASE_DIR="${SCRIPT_DIR}/topomaps/images"

usage()
{
    printf '%s\n' \
      'Usage: ./visualize_whill.sh --topomap-dir ROUTE [visualization options]' \
      '' \
      'Waypoint projection stays off unless --display-waypoints is supplied.'
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi

exec python3 "${SCRIPT_DIR}/placenav/visualization_node.py" \
    --robot whill \
    --robot-config-path "${ROBOT_CONFIG}" \
    --topomap-base-dir "${TOPOMAP_BASE_DIR}" \
    --cam-cal-path "${CAMERA_CALIBRATION}" \
    --camera-config-path "${CAMERA_CONFIG}" \
    "$@"
