#!/usr/bin/env bash

# Copyright 2022 Advanced Remanufacturing and Technology Centre
# Copyright 2022 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Function: Start Export Docker Container and run exporter.
# Static Analysis: shellcheck run_exporter.bash -x -e SC1091

EXPORT_MASKRCNN=$1

export_script="./home/user/trainer/exporter_files/scripts/run_op.bash"

if $EXPORT_MASKRCNN ; then
    CONTAINER_NAME="epd_p3_exporter"
else
    CONTAINER_NAME="epd_p2_exporter"
fi

sudo docker start $CONTAINER_NAME
sudo docker exec -it $CONTAINER_NAME "$export_script" "$EXPORT_MASKRCNN"
