#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
TOPOMAP_DIR="${SCRIPT_DIR}/topomaps/images"

usage()
{
    printf '%s\n' \
      'Usage: ./create_topomap_whill.sh ROUTE_NAME SAMPLE_PERIOD_SECONDS' \
      '' \
      'Run while a bag publishes /usb_cam/image_raw and /odom.' \
      'Example: ./create_topomap_whill.sh whill_short_route 5.0'
}

if [[ $# -ne 2 ]]; then
    usage
    exit 2
fi

exec python3 "${SCRIPT_DIR}/placenav/create_topomap.py" \
    --robot whill \
    --robot_config_path "${ROBOT_CONFIG}" \
    --route_name "$1" \
    --dt "$2" \
    --sampling-mode adaptive \
    --straight-distance 3.0 \
    --turn-distance 0.15 \
    --yaw-threshold 0.12 \
    --min-distance 0.10 \
    --topomap_directory "${TOPOMAP_DIR}"
