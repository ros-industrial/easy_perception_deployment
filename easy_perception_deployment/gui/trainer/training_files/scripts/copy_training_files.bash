#!/usr/bin/env bash

copy_script="./home/user/trainer/training_files/scripts/copy_op.bash"

docker start epd_p3_trainer
docker exec -it epd_p3_trainer $copy_script


