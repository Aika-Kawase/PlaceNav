#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
TOPOMAP_DIR="${SCRIPT_DIR}/topomaps/images"

usage()
{
    printf '%s\n' \
      'Usage: ./create_topomap_whill.sh ROUTE_NAME [SAMPLE_PERIOD_SECONDS] [options]' \
      '' \
      'Default: legacy map, adaptive sampling, /odom input, 3.0 m straight spacing.' \
      'The optional second positional argument is seconds, not meters.' \
      'Example: ./create_topomap_whill.sh route --map-mode odometry --straight-distance 2.0' \
      'Options: --odom-topic TOPIC, --sampling-mode time|adaptive, --idle-timeout SECONDS' \
      '         --max-node-distance METERS (odometry/adaptive default: 2*straight-distance)' \
      'Run while images and odometry are published. Ctrl+C finishes the map.'
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    usage
    exit 0
fi

ROUTE_NAME="$1"
shift
SAMPLE_PERIOD=3.0
if [[ $# -gt 0 && "$1" != -* ]]; then
    SAMPLE_PERIOD="$1"
    shift
fi

exec python3 "${SCRIPT_DIR}/placenav/create_topomap.py" \
    --robot whill \
    --robot_config_path "${ROBOT_CONFIG}" \
    --route_name "${ROUTE_NAME}" \
    --dt "${SAMPLE_PERIOD}" \
    --sampling-mode adaptive \
    --straight-distance 3.0 \
    --turn-distance 0.15 \
    --yaw-threshold 0.12 \
    --min-distance 0.10 \
    --topomap_directory "${TOPOMAP_DIR}" \
    "$@"
