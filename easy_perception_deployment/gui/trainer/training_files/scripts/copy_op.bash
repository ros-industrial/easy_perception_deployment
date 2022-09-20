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

# Function: Archive past custom weights folders within docker container.
# Copy over custom_dataset and training yaml files to specific folder in container.
# Static Analysis: shellcheck copy_op.bash -x -e SC1091

TRAIN_MASKRCNN=$1

if $TRAIN_MASKRCNN ; then
	cd "/home/user/p3_trainer/" || exit
else
	cd "/home/user/p2_trainer/" || exit
fi

# Check if there are trained checkpoints of previous training runs. 
# If true, rename weights/custom subdirectory in build farm.
if [ -d "weights/custom" ]; then
    CURRENTDATE=$(date +"%Y-%m-%d-%T")
    mv weights/custom weights/archived-on-"${CURRENTDATE}"
	mkdir -p weights/custom
fi
unset CURRENTDATE

# Copy over custom_dataset
if [ ! -d "/home/user/trainer/training_files/custom_dataset" ] ; then
  	echo "[ERROR] - [ custom_dataset ] NOT FOUND..."
else
	if $TRAIN_MASKRCNN ; then
		rm -r /home/user/p3_trainer/datasets/custom_dataset
		echo "[ custom_dataset ] FOUND. Transferring to p3_trainer/datasets/custom_dataset"
		cp --force -r /home/user/trainer/training_files/custom_dataset /home/user/p3_trainer/datasets/custom_dataset
	else
		rm -r /home/user/p2_trainer/datasets/custom_dataset
		echo "[ custom_dataset ] FOUND. Transferring to p2_trainer/datasets/custom_dataset"
		cp --force -r /home/user/trainer/training_files/custom_dataset /home/user/p2_trainer/datasets/custom_dataset
	fi
fi

if $TRAIN_MASKRCNN ; then
	# Copy over maskrcnn_training.yaml
	cp --force /home/user/trainer/training_files/maskrcnn_training.yaml /home/user/p3_trainer/configs/custom/maskrcnn_training.yaml
else
	# Copy over fasterrcnn_training.yaml
    cp --force /home/user/trainer/training_files/fasterrcnn_training.yaml /home/user/p2_trainer/configs/custom/fasterrcnn_training.yaml
fi