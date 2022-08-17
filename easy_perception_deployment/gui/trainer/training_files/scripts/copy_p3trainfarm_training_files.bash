#!/usr/bin/env bash

cd /home/user/maskrcnn-benchmark/

# Copy over custom_dataset
if [ ! -d "/home/user/trainer/training_files/custom_dataset" ] ; then
  	echo "[ERROR] - [ custom_dataset ] NOT FOUND..."
else
	echo "[ custom_dataset ] FOUND. Transferring to maskrcnn-benchmark/datasets/custom_dataset"
	cp --force -r /home/user/trainer/training_files/custom_dataset datasets/custom_dataset
fi

# Copy over maskrcnn_training.yaml
cp --force /home/user/trainer/training_files/maskrcnn_training.yaml configs/custom/maskrcnn_training.yaml

