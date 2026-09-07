#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "${SCRIPT_DIR}/placenav/gnm_path_adapter.py" \
    --input-topic /toponav/viz_info \
    --output-topic /placenav/gnm_path \
    --source-frame base_link \
    --target-frame odom \
    "$@"
