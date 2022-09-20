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

# Function: Start Train Docker Container and install necessary training dependencies.
# Static Analysis: shellcheck prepare_trainfarm_docker_container.bash -x -e SC1091

TRAIN_MASKRCNN=$1

install_script="./home/user/trainer/training_files/scripts/install_op.bash"

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else
    CONTAINER_NAME="epd_p2_trainer"
fi

sudo docker start $CONTAINER_NAME
sudo docker exec -it $CONTAINER_NAME "$install_script" "$TRAIN_MASKRCNN"
# sudo docker exec -it $CONTAINER_NAME ls /home/user/trainer/training_files/scripts/