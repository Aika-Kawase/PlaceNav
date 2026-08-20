#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/topomaps/bags"

usage()
{
    printf '%s\n' \
      'Usage: ./record_bag_whill.sh BAG_NAME' \
      '' \
      'WHILL, usb_cam, odom_relay, MotionDecision, and roscore must already run.' \
      'This script only records topics and never publishes a velocity command.'
}

if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi

mkdir -p "${BAG_DIR}"
cd "${BAG_DIR}"

exec rosbag record \
    /usb_cam/image_raw \
    /usb_cam/camera_info \
    /odom \
    /whill/odom \
    /local_path/cmd_vel \
    /whill/controller/cmd_vel \
    /joy \
    /emergency_stop \
    /tf \
    /tf_static \
    -O "$1"
