#!/usr/bin/env bash

install_script="./home/user/trainer/exporter_files/scripts/install_p3exporter_dependencies.bash"

docker start epd_p3_exporter
docker exec -it epd_p3_exporter $install_script
