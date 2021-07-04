#!/bin/bash
 
# Source ROS and Catkin workspaces
TALOS_SETUP=/talos_public_ws/devel/setup.bash
source /opt/ros/melodic/setup.bash
if [ -f $TALOS_SETUP ]
then
  source $TALOS_SETUP
fi
echo "Sourced Catkin workspace!"
 
# Execute the command passed into this entrypoint
exec "$@"
