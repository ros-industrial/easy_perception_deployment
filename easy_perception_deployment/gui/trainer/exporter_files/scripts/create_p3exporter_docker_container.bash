#!/usr/bin/env bash

# DEBUG STARTDIR = ./easy_perception_deployment/gui
docker create -it \
--name epd_p3_exporter \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/home/user/ \
--gpus all \
--shm-size 6g \
-u 0  \
cardboardcode/epd-p3-exporter
