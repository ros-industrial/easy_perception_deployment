#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

train_script="./home/user/trainer/training_files/scripts/run_op.bash $TRAIN_MASKRCNN"

docker start epd_p3_trainer
docker exec -it epd_p3_trainer $train_script