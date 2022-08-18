#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

if $TRAIN_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_trainer"
else
    CONTAINER_NAME="epd_p2_trainer"
fi

# STARTDIR = ./easy_perception_deployment/gui
docker create -it \
--name $CONTAINER_NAME \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/home/user/ \
--gpus all \
--shm-size 6g \
-u 0  \
cardboardcode/epd-trainer
