#!/usr/bin/env bash
# [parrot, sudo] Flash JetPack 5.1.2 to the AGX Xavier eMMC. ERASES the Xavier.
#
# Recovery mode: power off, connect the front USB-C port to parrot, hold the
# middle (Force Recovery) button, press Power, release both after ~2 s.
set -euo pipefail
source "$(dirname "$0")/common.sh"

[[ ${EUID} -eq 0 ]] || die "run with sudo"
[[ -x ${L4T_DIR}/flash.sh && -d ${L4T_DIR}/rootfs/home/${XAVIER_USER} ]] || die "run sudo ./20_prepare_rootfs.sh first"
lsusb -d 0955:7019 >/dev/null || die "no AGX Xavier in recovery mode on USB (lsusb 0955:7019)"

echo "This erases the Xavier eMMC and installs Jetson Linux R${L4T_RELEASE} (${BOARD})."
read -r -p "Type 'flash' to continue: " answer
[[ ${answer} == flash ]] || die "cancelled"

cd "${L4T_DIR}"
./flash.sh "${BOARD}" "${BOOT_DEVICE}"
echo "Flashed. After boot: ssh ${XAVIER_USER}@${XAVIER_IP} 'sudo nvpmodel -m 0' then ./40_build_image.sh"
