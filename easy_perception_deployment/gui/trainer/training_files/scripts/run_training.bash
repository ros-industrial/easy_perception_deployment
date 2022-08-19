#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

train_script="./home/user/trainer/training_files/scripts/run_op.bash $TRAIN_MASKRCNN"

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else
    CONTAINER_NAME="epd_p2_trainer"
fi

docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME $train_script
