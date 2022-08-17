#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

# Copy over custom_dataset to training_files/
cp -r ../data/datasets/custom_dataset trainer/training_files/

install_script="./home/user/trainer/training_files/scripts/install_op.bash $TRAIN_MASKRCNN"

docker start epd_p3_trainer
docker exec -it epd_p3_trainer $install_script