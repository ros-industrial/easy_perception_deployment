#!/usr/bin/env bash

EXPORT_MASKRCNN=$1

export_script="./home/user/trainer/exporter_files/scripts/run_op.bash $EXPORT_MASKRCNN"

if $EXPORT_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_exporter"
else
    CONTAINER_NAME="epd_p2_exporter"
fi

docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME $export_script
