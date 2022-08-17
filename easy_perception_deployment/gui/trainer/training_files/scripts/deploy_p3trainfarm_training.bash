#!/usr/bin/env bash

train_script="./home/user/trainer/training_files/scripts/run_p3trainfarm_training.bash"

docker start epd_p3_trainer
docker exec -it epd_p3_trainer $train_script
