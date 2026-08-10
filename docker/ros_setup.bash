# ROS 2 and Quattro workspace environment for interactive container shells.
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f "/ws/install/setup.bash" ]; then
    source "/ws/install/setup.bash"
fi
