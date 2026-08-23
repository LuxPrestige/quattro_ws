# ROS 2 and Quattro workspace environment for interactive container shells.
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f "/quattro_ws/install/setup.bash" ]; then
    source "/quattro_ws/install/setup.bash"
fi
