#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-/home/user/ws}"

source /opt/ros/noetic/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u

package_paths=(
  "${workspace}/src/teb_local_planner"
  "${workspace}/src/spatio_temporal_voxel_layer"
  "${workspace}/src/PlaceNav/src/placenav_teb_bridge"
  "${workspace}/src/UniDepth/ros1/unidepth_ros"
)

for package_path in "${package_paths[@]}"; do
  if [[ ! -f "${package_path}/package.xml" ]]; then
    echo "Missing ROS package: ${package_path}/package.xml" >&2
    exit 1
  fi
done

cd "${workspace}"
colcon build \
  --base-paths "${package_paths[@]}" \
  --packages-select \
    teb_local_planner \
    spatio_temporal_voxel_layer \
    placenav_teb_bridge \
    unidepth_ros \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --event-handlers console_direct+

echo
echo "Build completed. Run:"
echo "  source ${workspace}/install/setup.bash"
