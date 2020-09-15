#!/usr/bin/env bash

msg1="Sourcing [ROS2]"
msg2="Sourcing [Local Package/Workspace]"
msg3="Deploying package."
# Source ROS Eloquent
# TODO Run check if folder exist.
echo $msg1
source /opt/ros/eloquent/setup.bash

echo $msg2
# Check if the current easy_perception workspace has been built or not.
# If true, run selective colcon build.
# Otherwise, pass
cd ../
# echo "Building epd_msgs package"
# Source the epd_msgs workspace
cd ../epd_msgs
colcon build
# if [ ! -d "build" ] || [ ! -d "install" ] || [ ! -d "log" ] ; then
#   echo "ROS2 Package/Workspace has not been built. Running colcon build."
#   colcon build
# else
#   :
# fi
source install/setup.bash

# Source the main workspace
cd ../easy_perception_deployment
colcon build
# Source the workspace
# if [ ! -d "build" ] || [ ! -d "install" ] || [ ! -d "log" ] ; then
#   echo "ROS2 Package/Workspace has not been built. Running colcon build."
#   colcon build
# else
#   :
# fi
source install/setup.bash


# Launch easy_perception_deployment.
echo $msg3
ros2 launch easy_perception_deployment run.launch.py
