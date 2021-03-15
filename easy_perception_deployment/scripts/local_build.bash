# Source local ROS2 distro
is_bionic=$(cat /etc/issue.net | grep 20)

# If the Ubuntu OS is 18.04,
# Source ROS2 elqouent. Otherwise, source ROS2 Foxy.
if [ -z "$is_bionic" ]
then
    source /opt/ros/eloquent/setup.bash
else
    source /opt/ros/foxy/setup.bash
fi

unset is_bionic

# Build epd_msgs ROS2 package.
cd ../epd_msgs/
sudo rm -rf build/ install/ log/
echo "Building and Sourcing [ epd_msgs ]"
colcon build && source install/setup.bash

# Build easy_perception_deployment ROS2 package.
cd ../easy_perception_deployment/
sudo rm -rf build/ install/ log/
echo "Building and Sourcing [ easy_perception_deployment ]"
colcon build && source install/setup.bash
