#!/usr/bin/env bash

msg1="Sourcing [ROS2]"
msg2="Sourcing [Local Package/Workspace]"
msg3="Deploying package."

SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd $SCRIPTPATH

# Source ROS Foxy
# TODO Run check if folder exist.
echo $msg1
source /opt/ros/foxy/setup.bash

echo $msg2
# Check if the current easy_perception workspace has been built or not.
# If true, run selective colcon build.
# Otherwise, pass
cd ../../
echo "Building epd_msgs package"
# Source the epd_msgs workspace
cd ../epd_msgs
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ] ; then
  rm -r build install log
fi
colcon build
source install/setup.bash

# Source the main workspace
cd ../easy_perception_deployment
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ] ; then
  rm -r build install log
fi
colcon build
source install/setup.bash


# Launch easy_perception_deployment.
echo $msg3
ros2 launch easy_perception_deployment run.launch.py
