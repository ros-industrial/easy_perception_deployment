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

# Function: Install dependencies within Train Docker Container.
# Static Analysis: shellcheck install_op.bash -x -e SC1091

TRAIN_MASKRCNN=$1

cd /home/user/ || exit
if $TRAIN_MASKRCNN ; then
    git clone https://github.com/facebookresearch/maskrcnn-benchmark.git --depth 1 --single-branch p3_trainer
    cd p3_trainer || exit
else
    git clone https://github.com/facebookresearch/maskrcnn-benchmark.git --depth 1 --single-branch p2_trainer
    cd p2_trainer || exit
fi

export INSTALL_DIR=$PWD
git clone https://github.com/cardboardcode/cocoapi.git
cd cocoapi/PythonAPI || exit
python setup.py build_ext install
cd "$INSTALL_DIR" || exit
git clone https://github.com/cardboardcode/cityscapesScripts.git
cd cityscapesScripts || exit
python setup.py build_ext install
cd "$INSTALL_DIR" || exit
git clone https://github.com/cardboardcode/apex.git
cd apex  && python setup.py install
cd "$INSTALL_DIR" || exit
cp /home/user/trainer/training_files/modified_imports.py maskrcnn_benchmark/utils/imports.py
cp /home/user/trainer/training_files/modified_paths_catalog.py maskrcnn_benchmark/config/paths_catalog.py
python setup.py build install
# Create customs folder under config directory if does not exist.
if [ ! -d "configs/custom" ]; then
    mkdir -p configs/custom
fi
# Create weights folder if does not exist.
if [ ! -d "weights" ]; then
    mkdir weights
fi
cd weights || exit
if $TRAIN_MASKRCNN ; then
    FILE="e2e_mask_rcnn_R_50_FPN_1x.pth"
    if [ ! -f "$FILE" ]; then
        echo "Downloading MaskRCNN pretrained weights."
        wget https://download.pytorch.org/models/maskrcnn/e2e_mask_rcnn_R_50_FPN_1x.pth
    fi
else
    FILE="e2e_faster_rcnn_R_50_FPN_1x.pth"
    if [ ! -f "$FILE" ]; then   
        echo "Downloading FasterRCNN pretrained weights."
        wget https://download.pytorch.org/models/maskrcnn/e2e_faster_rcnn_R_50_FPN_1x.pth
    fi
fi
unset FILE
if $TRAIN_MASKRCNN ; then
    cp /home/user/trainer/training_files/trim_mask_rcnn.py trim_mask_rcnn.py
    python trim_mask_rcnn.py
else
    cp /home/user/trainer/training_files/trim_faster_rcnn.py trim_faster_rcnn.py
    python trim_faster_rcnn.py
fi
cd "$INSTALL_DIR" || exit
# Create datasets folder if does not exist.
if [ ! -d "datasets" ]; then
    mkdir datasets
fi
