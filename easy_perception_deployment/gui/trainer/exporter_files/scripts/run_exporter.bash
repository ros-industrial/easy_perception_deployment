#!/usr/bin/env bash

EXPORT_MASKRCNN=$1

export_script="./home/user/trainer/exporter_files/scripts/run_op.bash $EXPORT_MASKRCNN"

docker start epd_p3_exporter
docker exec -it epd_p3_exporter $export_script
