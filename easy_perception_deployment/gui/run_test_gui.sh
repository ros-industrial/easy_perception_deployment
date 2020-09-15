#!/bin/sh

# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

conda_installed=false
# Checking if Anaconda3 has been installed.
if output=$(conda --version > /dev/null 2>&1); then
    conda_installed=true
else
    echo "Please install Anaconda by refering to the installation docs."
    echo "[ https://docs.anaconda.com/anaconda/install/linux/ ]"
    exit 1
fi

# Checking if the epd_gui conda environment has been installed.
env_exists=$(conda env list | grep epd_gui)

if [ -z "$env_exists" ]
then
      echo "Installing epd_gui conda environment."
      conda env create -f epd_gui_env.yml
      eval "$(conda shell.bash hook)"
      conda activate epd_gui
      conda install pytest -y
      pip install pytest-qt
      pip install labelme
      conda deactivate
      echo "[epd_gui] env created."
fi

if $conda_installed; then
  eval "$(conda shell.bash hook)"
  conda activate epd_gui
  unset PYTHONPATH
  pytest --cov-report term-missing --cov=windows --cov=trainer test_gui.py
fi

unset conda_installed env_exists
