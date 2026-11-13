#!/usr/bin/env bash
# [Xavier] Start the inference container joined to parrot's ROS master.
#   ./run_xavier.sh                                   # interactive shell
#   ./run_xavier.sh python3 bench_inference.py --pr-models cosplace salad --wp-models omnivla_edge
set -euo pipefail
source "$(dirname "$0")/common.sh"
PLACENAV_DIR=$(cd "$(dirname "$0")/../.." && pwd)

interactive=()
[[ -t 0 ]] && interactive=(-it)
exec docker run --rm "${interactive[@]}" --runtime nvidia --network host --ipc host \
    -v "${PLACENAV_DIR}:/placenav" \
    -e "ROS_MASTER_URI=http://${PARROT_IP}:11311" -e "ROS_IP=${XAVIER_IP}" \
    "${IMAGE}" "$@"
