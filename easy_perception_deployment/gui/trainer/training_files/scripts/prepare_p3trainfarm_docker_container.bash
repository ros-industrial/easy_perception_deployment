#!/usr/bin/env bash

# Copy over custom_dataset to training_files/
cp -r ../data/datasets/custom_dataset trainer/training_files/

install_script="./home/user/trainer/training_files/scripts/install_p3trainfarm_dependencies.bash"

docker start epd_p3_trainer
docker exec -it epd_p3_trainer $install_script
