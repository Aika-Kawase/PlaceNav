#!/usr/bin/env bash
# [parrot, Internet] Download and verify the JetPack 5.1.2 BSP and sample rootfs.
set -euo pipefail
source "$(dirname "$0")/common.sh"

mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"
for pair in "${BSP_TARBALL}:${BSP_SHA256}" "${ROOTFS_TARBALL}:${ROOTFS_SHA256}"; do
    file=${pair%%:*}
    sum=${pair##*:}
    if [[ ! -f ${file} ]]; then
        curl -fL --retry 5 -o "${file}.part" "${L4T_URL}/${file}"
        mv "${file}.part" "${file}"
    fi
    echo "${sum}  ${file}" | sha256sum -c - || die "${file} checksum mismatch; delete it and retry"
done
echo "Downloaded to ${WORK_DIR}. Next: sudo ./20_prepare_rootfs.sh"
