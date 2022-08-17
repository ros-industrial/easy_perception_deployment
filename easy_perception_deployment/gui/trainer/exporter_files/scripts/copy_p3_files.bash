#!/usr/bin/env bash

copy_script="./home/user/trainer/exporter_files/scripts/copy_p3exporter_exporter_files.bash"

docker start epd_p3_exporter
docker exec -it epd_p3_exporter $copy_script


