
# **easy_perception_deployment**
[![Build Status](https://travis-ci.org/cardboardcode/easy_perception_deployment.svg?branch=master)](https://travis-ci.org/cardboardcode/easy_perception_deployment)

[![codecov](https://codecov.io/gh/cardboardcode/easy_perception_deployment/branch/master/graph/badge.svg)](https://codecov.io/gh/cardboardcode/easy_perception_deployment)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


## **What Is This?**

**easy_perception_deployment** is a ROS2 package that accelerates the training and deployment of CV models for industries.

To get started using it, please run the following commands to start **viewing the documentations**.
``` bash
cd $HOME
mkdir -p epd_ros2_ws/src && cd epd_ros2_ws/src
git clone https://gitlab.com/ROSI-AP/rosi-ap_rect/easy_perception_deployment

```

## **Setup**

This section lists steps on how to build **easy_perception_deployment** package using ROS2 build tools.

``` bash
# Download easy_perception_deployment
cd $HOME
mkdir -p epd_ros2_ws/src && cd epd_ros2_ws/src
git clone https://gitlab.com/ROSI-AP/rosi-ap_rect/easy_perception_deployment

# Install cv_bridge dependency
cd ../
sudo apt-get update
source /opt/ros/eloquent/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build packages and source them
cd src/easy_perception_deployment/epd_msgs/
colcon build && source install/setup.bash
cd ../easy_perception_deployment
colcon build && source install/setup.bash
```
