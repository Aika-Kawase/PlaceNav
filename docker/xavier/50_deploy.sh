#!/usr/bin/env bash
# [parrot -> Xavier over the direct link] Load the image and copy PlaceNav code,
# weights and topomaps. Re-run after code or topomap changes (the image only when rebuilt).
#   ./50_deploy.sh            # code + weights + topomaps (+ image if not loaded yet)
#   ./50_deploy.sh --image    # force re-loading the image
set -euo pipefail
source "$(dirname "$0")/common.sh"
PLACENAV_DIR=$(cd "$(dirname "$0")/../.." && pwd)
TARGET=${XAVIER_USER}@${XAVIER_IP}

ssh "${TARGET}" true || die "cannot ssh to ${TARGET}"
if [[ ${1:-} == --image ]] || ! ssh "${TARGET}" docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    [[ -f ${IMAGE_TAR} ]] || die "run ./40_build_image.sh first"
    gunzip -c "${IMAGE_TAR}" | ssh "${TARGET}" docker load
fi

# Sources, config, weights (incl. waypoint/sources and vpr exports) and topomaps.
# Generated feature DBs are rebuilt on the Xavier; bags stay on parrot.
rsync -a --info=progress2 \
    --exclude '__pycache__/' --exclude '*.bag' --exclude '.git/' \
    --exclude 'global-feats-*.h5' --exclude 'global-feats-*.h5.ready' \
    "${PLACENAV_DIR}/src" "${PLACENAV_DIR}/docker" "${TARGET}:PlaceNav/"
echo "Deployed to ${TARGET}:PlaceNav. On the Xavier: ~/PlaceNav/docker/xavier/run_xavier.sh"
