#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
# The devel space was moved after catkin_make, so use its Python path directly.
export PYTHONPATH="${PLACENAV_SRC}:/opt/placenav_viz_msgs_devel/lib/python3/dist-packages:${PYTHONPATH}"
cd "${PLACENAV_SRC}"
exec "$@"
