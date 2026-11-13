#!/usr/bin/env bash
# [Xavier, sudo] Run by 45_setup_xavier.sh. Adds, without touching the existing Docker:
#   1. /etc/fstab entry for the SSD (by UUID, nofail) so it survives reboots
#   2. NVIDIA container runtime debs (no nvidia-docker2; /etc/docker/daemon.json untouched)
#   3. a separate dockerd "docker-placenav" with data on the SSD and its own socket
#      (unix:///run/docker-placenav.sock) and containerd namespace
# Undo: sudo systemctl disable --now docker-placenav
#       sudo rm /etc/systemd/system/docker-placenav.service /etc/docker/placenav-daemon.json
#       remove the SSD line from /etc/fstab; optionally sudo dpkg -r the four packages.
set -euo pipefail
DEB_DIR=${1:?usage: xavier_host_setup.sh DEB_DIR}
XAVIER_SSD=${XAVIER_SSD:?}
XAVIER_PLACENAV_DIR=${XAVIER_PLACENAV_DIR:?}
PLACENAV_OWNER=${PLACENAV_OWNER:?}
die() { echo "ERROR: $*" >&2; exit 1; }

[[ ${EUID} -eq 0 ]] || die "run with sudo"
grep -q "R35 (release), REVISION: 1\." /etc/nv_tegra_release || die "expected L4T R35.1 (JetPack 5.0.2)"
[[ -e /etc/nvidia-container-runtime/host-files-for-container.d/l4t.csv ]] || die "l4t.csv missing"
mountpoint -q "${XAVIER_SSD}" || die "${XAVIER_SSD} is not mounted"

# 1. SSD mount at boot.
uuid=$(findmnt -n -o UUID --target "${XAVIER_SSD}")
fstype=$(findmnt -n -o FSTYPE --target "${XAVIER_SSD}")
[[ -n ${uuid} && ${fstype} == ext4 ]] || die "unexpected SSD filesystem: '${uuid}' '${fstype}'"
if ! grep -q "^UUID=${uuid}[[:space:]]" /etc/fstab; then
    backup=/etc/fstab.before-placenav.$(date +%Y%m%d%H%M%S)
    cp -a /etc/fstab "${backup}"
    echo "UUID=${uuid} ${XAVIER_SSD} ext4 defaults,nofail 0 2" >> /etc/fstab
    findmnt --verify --tab-file /etc/fstab >/dev/null || die "fstab check failed; restore ${backup}"
    echo "Added the SSD to /etc/fstab (backup: ${backup})"
fi

# 2. NVIDIA container runtime (CSV mode on Jetson: mounts the host driver listed in l4t.csv).
dpkg -i "${DEB_DIR}"/libnvidia-container0_*.deb "${DEB_DIR}"/libnvidia-container1_*.deb \
        "${DEB_DIR}"/libnvidia-container-tools_*.deb "${DEB_DIR}"/nvidia-container-toolkit_*.deb

# 3. Separate dockerd for PlaceNav; the default docker.service is neither restarted nor changed.
install -d -m 710 "${XAVIER_SSD}/docker-placenav"
install -d -o "${PLACENAV_OWNER}" -g "${PLACENAV_OWNER}" "${XAVIER_PLACENAV_DIR}"
install -d /etc/docker
cat > /etc/docker/placenav-daemon.json <<EOF
{
  "hosts": ["unix:///run/docker-placenav.sock"],
  "data-root": "${XAVIER_SSD}/docker-placenav",
  "exec-root": "/run/docker-placenav",
  "pidfile": "/run/docker-placenav.pid",
  "containerd": "/run/containerd/containerd.sock",
  "containerd-namespace": "placenav",
  "containerd-plugins-namespace": "placenav-plugins",
  "bridge": "none",
  "iptables": false,
  "ip6tables": false,
  "runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}},
  "default-runtime": "nvidia"
}
EOF
dockerd --validate --config-file /etc/docker/placenav-daemon.json

cat > /etc/systemd/system/docker-placenav.service <<EOF
[Unit]
Description=Docker daemon for PlaceNav (data on ${XAVIER_SSD}, separate from docker.service)
After=network-online.target containerd.service
Wants=network-online.target
Requires=containerd.service
RequiresMountsFor=${XAVIER_SSD}/docker-placenav

[Service]
Type=notify
ExecStart=/usr/bin/dockerd --config-file /etc/docker/placenav-daemon.json
ExecReload=/bin/kill -s HUP \$MAINPID
Restart=on-failure
RestartSec=2
LimitNOFILE=infinity
TasksMax=infinity
Delegate=yes
KillMode=process
OOMScoreAdjust=-500

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now docker-placenav.service

docker -H unix:///run/docker-placenav.sock info --format \
    'placenav dockerd: root={{.DockerRootDir}} runtimes={{range $k, $v := .Runtimes}}{{$k}} {{end}}default={{.DefaultRuntime}}'
echo "Existing Docker unchanged: $(systemctl is-active docker) (root: $(docker info --format '{{.DockerRootDir}}'))"
