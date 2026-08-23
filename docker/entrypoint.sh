#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f "/quattro_ws/install/setup.bash" ]; then
    source "/quattro_ws/install/setup.bash"
fi

exec "$@"
