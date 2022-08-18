#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

# Copy over custom_dataset to training_files/
cp -r ../data/datasets/custom_dataset trainer/training_files/

install_script="./home/user/trainer/training_files/scripts/install_op.bash $TRAIN_MASKRCNN"

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else
    CONTAINER_NAME="epd_p2_trainer"
fi

docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME $install_script
