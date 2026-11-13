#!/usr/bin/env bash
# [parrot, Internet, sudo] Build the Xavier root filesystem before flashing.
#
# The Xavier has no Internet, so everything it needs is installed here through a
# qemu chroot: Docker + NVIDIA container runtime (the inference image ships its
# own CUDA/cuDNN), chrony (time from parrot), a static IP on the parrot link, a
# user account (no first-boot wizard).
set -euo pipefail
source "$(dirname "$0")/common.sh"

[[ ${EUID} -eq 0 ]] || die "run with sudo"
[[ -f ${WORK_DIR}/${BSP_TARBALL} && -f ${WORK_DIR}/${ROOTFS_TARBALL} ]] || die "run ./10_download_bsp.sh first"
[[ ! -e ${L4T_DIR} ]] || die "${L4T_DIR} exists. Remove it (sudo rm -rf) to prepare a fresh rootfs."

read -r -s -p "Password for ${XAVIER_USER}@${XAVIER_HOSTNAME}: " PASSWORD; echo
read -r -s -p "Again: " PASSWORD2; echo
[[ -n ${PASSWORD} && ${PASSWORD} == "${PASSWORD2}" ]] || die "passwords are empty or differ"

cd "${WORK_DIR}"
tar xf "${BSP_TARBALL}"
tar xpf "${ROOTFS_TARBALL}" -C "${L4T_DIR}/rootfs"
cd "${L4T_DIR}"
./tools/l4t_flash_prerequisites.sh
./apply_binaries.sh
./tools/l4t_create_default_user.sh -u "${XAVIER_USER}" -p "${PASSWORD}" -n "${XAVIER_HOSTNAME}" --accept-license
unset PASSWORD PASSWORD2

ROOTFS=${L4T_DIR}/rootfs
mounts=()
cleanup() {
    for ((i = ${#mounts[@]} - 1; i >= 0; i--)); do umount -l "${mounts[i]}" || true; done
    rm -f "${ROOTFS}/usr/sbin/policy-rc.d" "${ROOTFS}/usr/bin/qemu-aarch64-static"
    if [[ -e ${ROOTFS}/etc/resolv.conf.placenav ]]; then
        rm -f "${ROOTFS}/etc/resolv.conf"
        mv "${ROOTFS}/etc/resolv.conf.placenav" "${ROOTFS}/etc/resolv.conf"
    fi
}
trap cleanup EXIT
cp /usr/bin/qemu-aarch64-static "${ROOTFS}/usr/bin/"
for dir in dev dev/pts proc sys; do
    mount --bind "/${dir}" "${ROOTFS}/${dir}"
    mounts+=("${ROOTFS}/${dir}")
done
mv "${ROOTFS}/etc/resolv.conf" "${ROOTFS}/etc/resolv.conf.placenav"   # usually a dangling symlink here
cp --dereference /etc/resolv.conf "${ROOTFS}/etc/resolv.conf"
printf '#!/bin/sh\nexit 101\n' > "${ROOTFS}/usr/sbin/policy-rc.d"   # never start services in the chroot
chmod 755 "${ROOTFS}/usr/sbin/policy-rc.d"

# The package postinst picks <SOC> from the board config, which is generic before flashing.
cat > "${ROOTFS}/etc/apt/sources.list.d/nvidia-l4t-apt-source.list" <<EOF
deb https://repo.download.nvidia.com/jetson/common r35.4 main
deb https://repo.download.nvidia.com/jetson/${SOC} r35.4 main
EOF
chroot "${ROOTFS}" /usr/bin/env -i PATH=/usr/sbin:/usr/bin:/sbin:/bin DEBIAN_FRONTEND=noninteractive LC_ALL=C \
    /bin/bash -euc "
        apt-get update
        apt-get install -y --no-install-recommends docker.io nvidia-container chrony
        usermod -aG docker '${XAVIER_USER}'
        apt-get clean
        rm -rf /var/lib/apt/lists/*
    "

mkdir -p "${ROOTFS}/etc/docker"
cat > "${ROOTFS}/etc/docker/daemon.json" <<'EOF'
{
  "runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}},
  "default-runtime": "nvidia"
}
EOF

connection=${ROOTFS}/etc/NetworkManager/system-connections/parrot-link.nmconnection
cat > "${connection}" <<EOF
[connection]
id=parrot-link
type=ethernet
interface-name=${XAVIER_IFACE}
autoconnect=true

[ipv4]
method=manual
address1=${XAVIER_IP}/${LINK_PREFIX}
never-default=true

[ipv6]
method=ignore
EOF
chmod 600 "${connection}"

grep -q "^server ${PARROT_IP} " "${ROOTFS}/etc/chrony/chrony.conf" || cat >> "${ROOTFS}/etc/chrony/chrony.conf" <<EOF

# PlaceNav: parrot is the only reachable clock. Step at any time if far off.
server ${PARROT_IP} iburst prefer
makestep 0.5 -1
EOF
grep -q "[[:space:]]parrot$" "${ROOTFS}/etc/hosts" || echo "${PARROT_IP} parrot" >> "${ROOTFS}/etc/hosts"
# MAXN is set once after first boot (nvpmodel -m 0 persists); /etc/nvpmodel.conf
# only exists after the board is identified at boot.

echo "Rootfs ready. Next: put the Xavier in recovery mode and run sudo ./30_flash.sh"
