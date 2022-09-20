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

# Function: Start and run copy_op.bash within Train Docker Container.
# Static Analysis: shellcheck copy_training_files.bash -x -e SC1091

TRAIN_MASKRCNN=$1

# Copy over custom_dataset to training_files/
sudo rm -r trainer/training_files/custom_dataset
cp -r ../data/datasets/custom_dataset trainer/training_files/

copy_script="./home/user/trainer/training_files/scripts/copy_op.bash"

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else 
    CONTAINER_NAME="epd_p2_trainer"
fi

sudo docker start $CONTAINER_NAME
sudo docker exec -it $CONTAINER_NAME /bin/sh "$copy_script" "$TRAIN_MASKRCNN"


