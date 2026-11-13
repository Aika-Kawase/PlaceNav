#!/usr/bin/env bash
# [parrot, Internet] Prepare the existing Xavier (JetPack 5.0.2, no reflash) for PlaceNav:
# download the NVIDIA container runtime debs, copy them with xavier_host_setup.sh,
# and run that script on the Xavier with sudo (asks for the Xavier password).
set -euo pipefail
source "$(dirname "$0")/common.sh"
TARGET=${XAVIER_USER}@${XAVIER_IP}
DEB_DIR=${WORK_DIR}/runtime_debs

mkdir -p "${DEB_DIR}"
for entry in "${RUNTIME_DEBS[@]}"; do
    read -r path sum <<<"${entry}"
    file=${DEB_DIR}/$(basename "${path}")
    [[ -f ${file} ]] || curl -fL --retry 5 -o "${file}" "${RUNTIME_DEB_URL}/${path}"
    echo "${sum}  ${file}" | sha256sum -c - || die "${file} checksum mismatch; delete it and retry"
done

ssh "${TARGET}" mkdir -p .placenav_setup
scp "${DEB_DIR}"/*.deb "$(dirname "$0")/xavier_host_setup.sh" "${TARGET}:.placenav_setup/"
ssh -t "${TARGET}" sudo XAVIER_SSD="${XAVIER_SSD}" XAVIER_PLACENAV_DIR="${XAVIER_PLACENAV_DIR}" \
    PLACENAV_OWNER="${XAVIER_USER}" bash .placenav_setup/xavier_host_setup.sh .placenav_setup
echo "Next: ./40_build_image.sh (if not done) and ./50_deploy.sh"
