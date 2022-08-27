#!/usr/bin/env bash

# Copyright 2022 Advanced Remanufacturing and Technology Centre
# Copyright 2022 ROS-Industrial Consortium Asia Pacific Team
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

# Function: Create Train Docker Container from Train Docker Image, epd-trainer 
# Static Analysis: shellcheck create_trainfarm_docker_container.bash -x -e SC1091

TRAIN_MASKRCNN=$1

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else
    CONTAINER_NAME="epd_p2_trainer"
fi

# STARTDIR = ./easy_perception_deployment/gui
sudo docker create -it \
--name $CONTAINER_NAME \
--env="QT_X11_NO_MITSHM=1" \
--env="FORCE_CUDA=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v "$(pwd)":/home/user/ \
--gpus all \
--shm-size 6g \
-u 0  \
cardboardcode/epd-trainer
