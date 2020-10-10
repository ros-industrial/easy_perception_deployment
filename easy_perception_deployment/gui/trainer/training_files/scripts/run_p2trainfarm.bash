#!/usr/bin/env bash

export START_DIR=$PWD

model_name=$1
timestamp=$2
hours_min=$(date | sed 's/.* \([0-9]*:[0-9]*\):[0-9]*.*/\1/')
path_to_dataset=$3
path_to_config=$4

cd trainer/P2TrainFarm
export OUTPUT_DIR=$PWD

cd maskrcnn-benchmark
export INSTALL_DIR=$PWD

# Checking if the p2_trainer conda environment has been installed.
env_exists=$(conda env list | grep p2_trainer)

if [ -z "$env_exists" ]
then
  echo "Installation of [p2_trainer] conda env failed. Please run install_p2trainfarm.sh."
else
  eval "$(conda shell.bash hook)"
  conda activate p2_trainer

  cd $START_DIR
  cp $path_to_config $INSTALL_DIR/configs/custom/fasterrcnn_training.yaml

  # Check if custom_dataset is not in build farm directory.
  # If true, just copy over.
  # If false, overwrite.
  cd $START_DIR
  if [ ! -d "$INSTALL_DIR/datasets/custom_dataset" ]; then
      cp -r $path_to_dataset $INSTALL_DIR/datasets/custom_dataset
  else
      rm -r $INSTALL_DIR/datasets/custom_dataset
      cp -rf $path_to_dataset $INSTALL_DIR/datasets/
  fi

  # Check if there are trained checkpoints of previous training runs.
  # If true, rename weights/custom subdirectory in build farm.
  # Otherwise, do nothing.
  if [ ! -d "$INSTALL_DIR/weights/custom" ]; then
      :
  else
      mv $INSTALL_DIR/weights/custom $INSTALL_DIR/weights/archived_on_"$timestamp"_"$hours_min"
  fi

  cd $INSTALL_DIR
  python3 tools/train_net.py --config-file configs/custom/fasterrcnn_training.yaml
  # Once training completes, transfer to outside build farms
  cd $OUTPUT_DIR
  cp maskrcnn-benchmark/weights/custom/model_0003000.pth trained_models/trained.pth

fi

unset OUTPUT_DIR INSTALL_DIR START_DIR
