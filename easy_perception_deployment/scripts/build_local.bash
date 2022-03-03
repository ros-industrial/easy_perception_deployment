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

# Function: Source all required setup.bash and build both epd_msgs easy_perception_deployment
# Static Analysis: shellcheck build_local.bash -x -e SC1091

# Sourcing [ ROS2 Foxy ]
if [ ! -f "/opt/ros/foxy/setup.bash" ]; then
  echo "ROS2 Foxy is not installed."
  echo "Please install it via this link: "
  echo "https://index.ros.org/doc/ros2/Installation/Foxy/"
  exit 1
else
  source /opt/ros/foxy/setup.bash
fi

# Build epd_msgs ROS2 package.

if [ ! -d "../epd_msgs/" ]; then
  echo "[ epd_msgs ] ROS2 package is missing."
  exit 1
else
  cd ../epd_msgs/ || exit
  if [ -d  "build/" ]; then
    sudo rm -rf build/
  fi
  if [ -d  "install/" ]; then
    sudo rm -rf install/
  fi
  if [ -d  "log/" ]; then
    sudo rm -rf log/
  fi
  echo "Building and Sourcing [ epd_msgs ]"
  colcon build && source install/setup.bash
  # TODO (cardboardcode): Put in safeguard against failed colcon build.
fi

# Build easy_perception_deployment ROS2 package.
cd ../easy_perception_deployment/ || exit
if [ -d  "build/" ]; then
  sudo rm -rf build/
fi
if [ -d  "install/" ]; then
  sudo rm -rf install/
fi
if [ -d  "log/" ]; then
  sudo rm -rf log/
fi
echo "Building and Sourcing [ easy_perception_deployment ]"
colcon build && source install/setup.bash
# TODO (cardboardcode): Put in safeguard against failed colcon build.
