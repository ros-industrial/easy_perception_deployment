# Source local ROS2 distro
is_bionic=$(cat /etc/issue.net | grep 20)

if [ -z "$is_bionic" ]
then
    source /opt/ros/eloquent/setup.bash
else
    source /opt/ros/foxy/setup.bash
fi

cd ../epd_msgs/
rm -r build/ install/ log/
echo "Building and Sourcing [ epd_msgs ]"
colcon build && source install/setup.bash

cd ../easy_perception_deployment/
rm -r build/ install/ log/
echo "Building and Sourcing [ easy_perception_deployment ]"
colcon build && source install/setup.bash
