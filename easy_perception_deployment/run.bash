#!/usr/bin/env bash

export PATH=~/anaconda3/bin:$PATH
PATH_TO_THIS_SCRIPT=$( realpath "$0"  )
START_DIR=$( dirname $PATH_TO_THIS_SCRIPT )

cd $START_DIR

# Check if Anaconda has been installed in general.
# If true, get the first digit of the string which should reflect the major version of conda.
if detected_conda=$(conda --version); then
    # echo $detected_conda - FOUND
    declare -i conda_ver
    conda_ver=$(echo $detected_conda | grep -o -E '[0-9]+' | head -1 | sed -e 's/^0\+//')
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

# Check if pretrained models have been downloaded.
P2FILE=./data/model/FasterRCNN-10.onnx
if [ ! -f "$P2FILE" ]; then
    echo "Downloading $P2FILE."
    wget \
    https://github.com/onnx/models/raw/main/vision/object_detection_segmentation/faster-rcnn/model/FasterRCNN-10.onnx \
    --directory-prefix=./data/model/
fi
unset P2FILE

P3FILE=./data/model/MaskRCNN-10.onnx
if [ ! -f "$P3FILE" ]; then
    echo "Downloading $P3FILE."
    wget \
    https://github.com/onnx/models/raw/main/vision/object_detection_segmentation/mask-rcnn/model/MaskRCNN-10.onnx \
    --directory-prefix=./data/model/
fi
unset P3FILE

# Checking if the epd_gui_env conda environment has been installed.
env_exists=$(conda env list | grep epd_gui_env)

if [ -z "$env_exists" ]
then
      echo "Installing epd_gui_env conda environment."
      conda create -n epd_gui_env python=3.6 -y
      eval "$(conda shell.bash hook)"
      conda activate epd_gui_env
      pip install PySide2==5.15.0
      pip install dateutils==0.6.12
      pip install pycocotools==2.0.2
      pip install labelme==5.0.1
      conda deactivate
      echo "[epd_gui_env] env created."
fi

eval "$(conda shell.bash hook)"
conda activate epd_gui_env
cd $PWD/gui
python main.py

unset START_DIR PATH_TO_THIS_SCRIPT env_exists
