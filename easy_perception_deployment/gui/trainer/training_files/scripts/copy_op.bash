#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

cd /home/user/maskrcnn-benchmark/

# Check if there are trained checkpoints of previous training runs.
  # If true, rename weights/custom subdirectory in build farm.
if [ -d "weights/custom" ]; then
    CURRENTDATE=`date +"%Y-%m-%d-%T"`
    mv weights/custom weights/archived-on-"${CURRENTDATE}"
	mkdir -p weights/custom
fi
unset CURRENTDATE

# Copy over custom_dataset
if [ ! -d "/home/user/trainer/training_files/custom_dataset" ] ; then
  	echo "[ERROR] - [ custom_dataset ] NOT FOUND..."
else
	echo "[ custom_dataset ] FOUND. Transferring to maskrcnn-benchmark/datasets/custom_dataset"
	cp --force -r /home/user/trainer/training_files/custom_dataset /home/user/maskrcnn-benchmark/datasets/custom_dataset
fi

if $TRAIN_MASKRCNN ; then
	# Copy over maskrcnn_training.yaml
	cp --force /home/user/trainer/training_files/maskrcnn_training.yaml /home/user/maskrcnn-benchmark/configs/custom/maskrcnn_training.yaml
else
	# Copy over fasterrcnn_training.yaml
    cp --force /home/user/trainer/training_files/fasterrcnn_training.yaml /home/user/maskrcnn-benchmark/configs/custom/fasterrcnn_training.yaml
fi


