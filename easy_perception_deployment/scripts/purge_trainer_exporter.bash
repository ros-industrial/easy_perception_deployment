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

# Function: Removes docker images, containers and local folders with files concerning P3 and P2 training and exporting. 
# Static Analysis: shellcheck purge_trainer_exporter.bash -x -e SC1091

echo "Removing [/p3_trainer], [/p2_trainer], [/p3_exporter], [/p2_exporter] local folders"
read -rp "Are you sure? [y/n]: " input

if [[ $input == "y" ]]; then
  if [ -d "gui/p3_trainer" ]; then
    sudo rm -r "gui/p3_trainer" 
  fi 
  if [ -d "gui/p3_exporter" ]; then 
    sudo rm -r "gui/p3_exporter"
  fi
  if [ -d "gui/p2_trainer" ]; then 
    sudo rm -r "gui/p2_trainer"
  fi
  if [ -d "gui/p2_exporter" ]; then 
   sudo rm -r "gui/p2_exporter"
  fi
elif [[ $input == "n" ]]; then
  :
else
  echo "Invalid input received. Skipping..."
fi

echo "Removing transient [trained.pth], [output.onnx], [/trainer/training/custom_dataset] local files"
read -rp "Are you sure? [y/n]: " input

if [[ $input == "y" ]]; then
  if [ -d "gui/trained.pth" ]; then
    sudo rm gui/trained.pth
  fi
  if [ -d "gui/output.onnx" ]; then
    sudo rm gui/output.onnx
  fi 
  if [ -d "gui/trainer/training_files/custome_dataset" ]; then
    sudo rm -r gui/trainer/training_files/custom_dataset
  fi 
elif [[ $input == "n" ]]; then
  :
else
  echo "Invalid input received. Skipping..."
fi

echo "Removing [epd_p3_trainer], [epd_p3_exporter], [epd_p2_trainer], [epd_p2_exporter] Docker Containers"
read -rp "Are you sure? [y/n]: " input

if [[ $input == "y" ]]; then
  docker stop epd_p3_trainer epd_p3_exporter epd_p2_trainer epd_p2_exporter
  docker container rm epd_p3_trainer epd_p3_exporter epd_p2_trainer epd_p2_exporter
elif [[ $input == "n" ]]; then
  :
else
  echo "Invalid input received. Skipping..."
fi


echo "Removing [epd-trainer], [epd-exporter] Docker Images"
read -rp "Are you sure? [y/n]: " input

if [[ $input == "y" ]]; then
  docker image rm cardboardcode/epd-trainer cardboardcode/epd-exporter
elif [[ $input == "n" ]]; then
  :
else
  echo "Invalid input received. Skipping..."
fi
