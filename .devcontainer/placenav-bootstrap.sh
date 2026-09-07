#!/usr/bin/env bash
set -euo pipefail

WS=/home/user/ws
PLACE_NAV="$WS/src/PlaceNav"
WEIGHTS="$PLACE_NAV/src/model_weights"
THIRD_PARTY="$WS/third_party/drive-any-robot"

mkdir -p "$WEIGHTS" "$WS/third_party"
python3 -m pip install --user --disable-pip-version-check gdown

if [ ! -f "$WEIGHTS/gnm_large.pth" ]; then
  python3 -m gdown "https://drive.google.com/uc?id=1WluphDqeTY5eTGp98Nkc3St6fr9PyrA5" -O "$WEIGHTS/gnm_large.pth"
fi
if [ ! -f "$WEIGHTS/efficientnet_85x85.pth" ]; then
  python3 -m gdown "https://drive.google.com/uc?id=1M1rvlRYiV9F0VHKggAOyzun4PwGFTlZ1" -O "$WEIGHTS/efficientnet_85x85.pth"
fi

if [ ! -d "$THIRD_PARTY/train/gnm_train" ]; then
  tmp_home=$(mktemp -d)
  HOME="$tmp_home" GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null GIT_CONFIG_NOSYSTEM=1 \
    git clone https://github.com/robodhruv/drive-any-robot.git "$THIRD_PARTY"
  rm -rf "$tmp_home"
fi
python3 -m pip install --user --no-deps -e "$THIRD_PARTY/train"

if [ "${BUILD_OBSTACLE_STACK:-0}" = "1" ]; then
  "$PLACE_NAV/scripts/build_obstacle_stack_ros1.sh" "$WS"
fi

exec "$@"
