#!/usr/bin/env bash

export PATH=~/anaconda3/bin:$PATH
PATH_TO_THIS_SCRIPT=$( realpath "$0"  )
START_DIR=$( dirname $PATH_TO_THIS_SCRIPT )

cd $START_DIR

# Check if Anaconda has been installed in general.
# If true, get the first digit of the string which should reflect the major version of conda.
if output=$(conda --version); then
    echo $output
    declare -i conda_ver
    conda_ver=$(echo $output | grep -o -E '[0-9]+' | head -1 | sed -e 's/^0\+//')
else
    echo "Please install Anaconda by refering to the installation docs."
    echo "Exiting terminal in 10 seconds."
    echo "[ https://docs.anaconda.com/anaconda/install/linux/ ]"
    sleep 10
    exit 1
fi

# Check if Anaconda is Anaconda2 or below.
if (( conda_ver < 2 )); then
    echo "Anaconda3 - NOTFOUND. Please install Anaconda3."
    echo "Exiting terminal in 10 seconds."
    sleep 10
    exit 1
fi

# Checking if the epd_gui conda environment has been installed.
env_exists=$(conda env list | grep epd_gui)

if [ -z "$env_exists" ]
then
      echo "Installing epd_gui conda environment."
      conda env create -f gui/epd_gui_env.yml
      eval "$(conda shell.bash hook)"
      conda activate epd_gui
      conda install pytest -y
      pip install pytest-qt
      pip install labelme
      pip install lark-parser
      pip install pyside2
      pip install empy
      conda deactivate
      echo "[epd_gui] env created."
fi

eval "$(conda shell.bash hook)"
conda activate epd_gui
cd $PWD/gui
python main.py

unset START_DIR PATH_TO_THIS_SCRIPT env_exists
