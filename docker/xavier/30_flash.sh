#!/usr/bin/env bash
# [parrot, sudo] Flash JetPack 5.1.2 to the AGX Xavier eMMC. ERASES the Xavier.
#
# Recovery mode (USB-C flashing port connected to parrot, no hub):
#   running:   hold Force Recovery (middle), press+release Reset (right), release after ~2 s
#   off:       hold Force Recovery (middle), press+release Power (left), release after ~2 s
# lsusb then shows 0955:7019 (0955:7020 means it booted normally instead).
set -euo pipefail
source "$(dirname "$0")/common.sh"

[[ ${EUID} -eq 0 ]] || die "run with sudo"
[[ -x ${L4T_DIR}/flash.sh && -d ${L4T_DIR}/rootfs/home/${XAVIER_USER} ]] || die "run sudo ./20_prepare_rootfs.sh first"
if ! lsusb -d 0955:7019 >/dev/null; then
    echo "NVIDIA USB devices seen by parrot:" >&2
    lsusb -d 0955: >&2 || echo "  (none: check the cable is a data cable and the Xavier's flashing USB-C port)" >&2
    if lsusb -d 0955:7020 >/dev/null; then
        echo "0955:7020 = the Xavier booted normally. Hold Force Recovery (middle), press and release" \
             "Reset (right), release Force Recovery after ~2 s, then rerun." >&2
    fi
    die "no AGX Xavier in recovery mode on USB (expected lsusb 0955:7019)"
fi

echo "This erases the Xavier eMMC and installs Jetson Linux R${L4T_RELEASE} (${BOARD})."
read -r -p "Type 'flash' to continue: " answer
[[ ${answer} == flash ]] || die "cancelled"

cd "${L4T_DIR}"
./flash.sh "${BOARD}" "${BOOT_DEVICE}"
echo "Flashed. After boot: ssh ${XAVIER_USER}@${XAVIER_IP} 'sudo nvpmodel -m 0' then ./40_build_image.sh"
