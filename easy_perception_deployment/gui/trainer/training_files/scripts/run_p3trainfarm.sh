#!/usr/bin/env bash

model_name=$1
timestamp=$2

cd trainer/P3TrainFarm

export OUTPUT_DIR=$PWD

cd maskrcnn-benchmark

# Checking if the p3_trainer conda environment has been installed.
env_exists=$(conda env list | grep p3_trainer)

if [ -z "$env_exists" ]
then
  echo "Installation of [p3_trainer] conda env failed. Please run install_p3trainfarm.sh."
else
  eval "$(conda shell.bash hook)"
  conda activate p3_trainer
  python tools/train_net.py --config-file configs/custom/maskrcnn_training.yaml
  # Once training completes, transfer to outside build farms
  cd $OUTPUT_DIR
  cp maskrcnn-benchmark/weights/custom/model_0003000.pth trained_models/trained.pth

fi
