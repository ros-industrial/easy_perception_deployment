#!/usr/bin/env bash

export_script="./home/user/trainer/exporter_files/scripts/run_p3exporter_exporter.bash"

docker start epd_p3_exporter
docker exec -it epd_p3_exporter $export_script
