#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/placenav/safety_filter.py" \
  --robot whill \
  --robot-config-path "${SCRIPT_DIR}/config/robots.yaml"
