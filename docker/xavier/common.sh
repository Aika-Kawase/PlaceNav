# Shared settings for the Xavier installer. Sourced by the numbered scripts.
# Override any value through the environment, e.g. XAVIER_IP=192.168.1.10 ./20_prepare_rootfs.sh
# shellcheck shell=bash

# JetPack 5.1.2 = Jetson Linux R35.4.1 (last JetPack line that supports AGX Xavier).
L4T_RELEASE=35.4.1
L4T_URL=https://developer.download.nvidia.com/embedded/L4T/r35_Release_v4.1/release
BSP_TARBALL=Jetson_Linux_R35.4.1_aarch64.tbz2
BSP_SHA256=72b75a0c7fa3bf6ef41ae06634bb67c38a92682155d1206026dbee4a6b9a016f
ROOTFS_TARBALL=Tegra_Linux_Sample-Root-Filesystem_R35.4.1_aarch64.tbz2
ROOTFS_SHA256=27656df9aa7d0171905d8da18197cb2bc5225e12bcd8a9abda3922b5eba94ccd
BOARD=jetson-agx-xavier-devkit
BOOT_DEVICE=mmcblk0p1   # internal eMMC
SOC=t194                # AGX Xavier

# Same directory with and without sudo (sudo would otherwise switch $HOME to /root).
INVOKING_HOME=$(getent passwd "${SUDO_USER:-$(id -un)}" | cut -d: -f6)
WORK_DIR=${WORK_DIR:-${INVOKING_HOME}/xavier_jp512}
L4T_DIR=${WORK_DIR}/Linux_for_Tegra

# Link between parrot and the Xavier (no Internet on that link).
XAVIER_IP=${XAVIER_IP:-192.168.1.246}
PARROT_IP=${PARROT_IP:-192.168.1.146}
LINK_PREFIX=${LINK_PREFIX:-24}
XAVIER_IFACE=${XAVIER_IFACE:-eth0}
XAVIER_USER=${XAVIER_USER:-amsl}
XAVIER_HOSTNAME=${XAVIER_HOSTNAME:-xavier}

# Inference image (built on parrot for linux/arm64, then copied to the Xavier).
# The base image is pinned in the Dockerfile.
IMAGE=${IMAGE:-placenav-xavier:r35.4.1}
IMAGE_TAR=${WORK_DIR}/placenav-xavier.tar.gz

die() { echo "ERROR: $*" >&2; exit 1; }
