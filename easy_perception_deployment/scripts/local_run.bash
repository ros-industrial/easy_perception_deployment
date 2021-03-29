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

# Run easy_perception_deployment ROS2 package.
source install/setup.bash
source ../epd_msgs/install/setup.bash
echo "Running [ easy_perception_deployment ]"
ros2 launch easy_perception_deployment run.launch.py
