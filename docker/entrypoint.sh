#!/bin/bash
# Basic entrypoint for ROS / catkin Docker containers

# Source ROS 
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /underlay_ws/devel/setup.bash ]
then
  source /underlay_ws/devel/setup.bash
  echo "Sourced underlay workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/devel/setup.bash ]
then
  source /overlay_ws/devel/setup.bash
  echo "Sourced overlay workspace"
fi

# Add Neptus folder to PATH
export PATH=/underlay_ws/src/neptus:$PATH

# Execute the command passed into this entrypoint
exec "$@"