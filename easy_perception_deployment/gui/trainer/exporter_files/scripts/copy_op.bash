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

# Function: Copy over custom_dataset and exporting yaml files to specific folder in container.
# Static Analysis: shellcheck copy_op.bash -x -e SC1091

EXPORT_MASKRCNN=$1

if $EXPORT_MASKRCNN ; then
	cd /home/user/p3_exporter || exit
else
	cd /home/user/p2_exporter || exit
fi
# Copy over custom_dataset
if [ ! -f "/home/user/trained.pth" ] ; then
  	echo "[ERROR] - [ trained.pth ] NOT FOUND..."
else
	echo "[ trained.pth ] FOUND. Transferring to p3_exporter/weights/custom/trained.pth"
	cp --force -r /home/user/trained.pth weights/custom/
fi

if $EXPORT_MASKRCNN ; then
	# Copy over maskrcnn_export.yaml
	echo "Transferring maskrcnn_export.yaml to p3_exporter/configs/custom/maskrcnn_export.yaml"
	cp --force /home/user/trainer/exporter_files/maskrcnn_export.yaml /home/user/p3_exporter/configs/custom/maskrcnn_export.yaml
else
	# Copy over faster_export.yaml
	echo "Transferring fasterrcnn_export.yaml to p3_exporter/configs/custom/fasterrcnn_export.yaml"
	cp --force /home/user/trainer/exporter_files/fasterrcnn_export.yaml /home/user/p2_exporter/configs/custom/fasterrcnn_export.yaml
fi