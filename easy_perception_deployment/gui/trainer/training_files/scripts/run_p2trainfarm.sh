#!/usr/bin/env bash

model_name=$1
timestamp=$2

cd trainer/P2TrainFarm

export OUTPUT_DIR=$PWD

cd maskrcnn-benchmark

# Checking if the p2_trainer conda environment has been installed.
env_exists=$(conda env list | grep p2_trainer)

if [ -z "$env_exists" ]
then
  echo "Installation of [p2_trainer] conda env failed. Please run install_p2trainfarm.sh."
else
  eval "$(conda shell.bash hook)"
  conda activate p2_trainer
  python tools/train_net.py --config-file configs/custom/fasterrcnn_training.yaml
  # Once training completes, transfer to outside build farms
  cd $OUTPUT_DIR
  cp maskrcnn-benchmark/weights/custom/model_0003000.pth trained_models/trained.pth

fi
