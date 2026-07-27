#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
MODEL_CONFIG="${SCRIPT_DIR}/config/models.yaml"
MODEL_WEIGHT_DIR="${SCRIPT_DIR}/model_weights"
TOPOMAP_BASE_DIR="${SCRIPT_DIR}/topomaps/images"

usage()
{
    printf '%s\n' \
      'Usage: ./navigate_whill.sh --topomap-dir ROUTE [PlaceNav options]' \
      '' \
      'Starts inference only. It never starts the PD controller or robot drivers.'
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi

if [[ ! -f "${MODEL_WEIGHT_DIR}/gnm_large.pth" ]]; then
    echo "Missing GNM checkpoint: ${MODEL_WEIGHT_DIR}/gnm_large.pth" >&2
    exit 1
fi

if [[ ! -f "${MODEL_WEIGHT_DIR}/efficientnet_85x85.pth" ]]; then
    echo "Missing CosPlace checkpoint: ${MODEL_WEIGHT_DIR}/efficientnet_85x85.pth" >&2
    exit 1
fi

exec python3 "${SCRIPT_DIR}/placenav/navigate.py" \
    --robot whill \
    --robot-config-path "${ROBOT_CONFIG}" \
    --topomap-base-dir "${TOPOMAP_BASE_DIR}" \
    --model-weight-dir "${MODEL_WEIGHT_DIR}" \
    --model-config-path "${MODEL_CONFIG}" \
    "$@"
