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
      'Starts inference and the GUI visualization. Use --no-visualization to suppress the GUI.' \
      'Starts the GNM Path adapter. Use --no-path-adapter to suppress it.' \
      'It never starts the PD controller or robot drivers.'
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

TOPOMAP_DIR=""
PR_MODEL="cosplace"
SHOW_VISUALIZATION=1
START_PATH_ADAPTER=1
NAV_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --topomap-dir)
            TOPOMAP_DIR="$2"
            NAV_ARGS+=("$1" "$2")
            shift 2
            ;;
        --pr-model)
            PR_MODEL="$2"
            NAV_ARGS+=("$1" "$2")
            shift 2
            ;;
        --no-visualization)
            SHOW_VISUALIZATION=0
            shift
            ;;
        --no-path-adapter)
            START_PATH_ADAPTER=0
            shift
            ;;
        *)
            NAV_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ -z "${TOPOMAP_DIR}" ]]; then
    echo "--topomap-dir is required" >&2
    exit 2
fi

DB_PATH="${TOPOMAP_BASE_DIR}/${TOPOMAP_DIR}/global-feats-${PR_MODEL}.h5"
DB_READY_PATH="${DB_PATH}.ready"
rm -f "${DB_READY_PATH}"

python3 "${SCRIPT_DIR}/placenav/navigate.py" \
    --robot whill \
    --robot-config-path "${ROBOT_CONFIG}" \
    --topomap-base-dir "${TOPOMAP_BASE_DIR}" \
    --model-weight-dir "${MODEL_WEIGHT_DIR}" \
    --model-config-path "${MODEL_CONFIG}" \
    "${NAV_ARGS[@]}" &
NAV_PID=$!
VIZ_PID=""
PATH_ADAPTER_PID=""

if [[ "${START_PATH_ADAPTER}" -eq 1 ]]; then
    "${SCRIPT_DIR}/run_gnm_path_adapter_whill.sh" &
    PATH_ADAPTER_PID=$!
fi

cleanup()
{
    [[ -z "${VIZ_PID}" ]] || kill "${VIZ_PID}" 2>/dev/null || true
    [[ -z "${PATH_ADAPTER_PID}" ]] || kill "${PATH_ADAPTER_PID}" 2>/dev/null || true
    kill "${NAV_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${SHOW_VISUALIZATION}" -eq 1 ]]; then
    VISUALIZATION_READY=0
    while kill -0 "${NAV_PID}" 2>/dev/null; do
        if [[ -f "${DB_READY_PATH}" ]] && rosnode ping -c 1 /PlaceNavNode >/dev/null 2>&1; then
            VISUALIZATION_READY=1
            break
        fi
        sleep 1
    done
    if [[ "${VISUALIZATION_READY}" -eq 1 ]] && kill -0 "${NAV_PID}" 2>/dev/null; then
        "${SCRIPT_DIR}/visualize_whill.sh" \
            --topomap-dir "${TOPOMAP_DIR}" \
            --pr-model "${PR_MODEL}" \
            --show &
        VIZ_PID=$!
    else
        echo "Visualization was not started: navigation exited or ${DB_READY_PATH} is missing." >&2
    fi
fi

set +e
wait "${NAV_PID}"
NAV_STATUS=$?
set -e
exit "${NAV_STATUS}"
