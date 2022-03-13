#!/usr/bin/env bash

# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Function: Source all required setup.bash and launch easy_perception_deployment
# Static Analysis: shellcheck run_local.bash -x -e SC1091

# Sourcing [ ROS2 Foxy ]
if [ ! -f "/opt/ros/foxy/setup.bash" ]; then
  echo "ROS2 Foxy is not installed."
  echo "Please install it via this link: "
  echo "https://index.ros.org/doc/ros2/Installation/Foxy/"
  exit 1
else
  source /opt/ros/foxy/setup.bash
fi

# Sourcing [ easy_perception_deployment ] package
if [ ! -f "install/setup.bash" ]; then
  echo "No ROS2 package/workspace detected."
  echo "Please ensure you have built this package by running colcon build."
  exit 1
else
  source install/setup.bash
  # Sourcing [ epd_msgs ] package
  if [ ! -f "../epd_msgs/install/setup.bash" ]; then
    echo "[ epd_msgs ] ROS2 package is missing."
    exit 1
  else
    source ../epd_msgs/install/setup.bash
  fi

fi
# Run easy_perception_deployment ROS2 package.
echo "Running [ easy_perception_deployment ]"
ros2 launch easy_perception_deployment run.launch.py
