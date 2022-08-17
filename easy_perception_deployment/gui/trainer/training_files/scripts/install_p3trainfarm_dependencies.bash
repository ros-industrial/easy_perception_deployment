#!/usr/bin/env bash

cd /home/user/
git clone https://github.com/facebookresearch/maskrcnn-benchmark.git --depth 1 --single-branch
cd maskrcnn-benchmark
export INSTALL_DIR=$PWD
git clone https://github.com/cardboardcode/cocoapi.git
cd cocoapi/PythonAPI
python setup.py build_ext install
cd $INSTALL_DIR
git clone https://github.com/cardboardcode/cityscapesScripts.git
cd cityscapesScripts
python setup.py build_ext install
cd $INSTALL_DIR
git clone https://github.com/cardboardcode/apex.git
cd apex  && python setup.py install
cd $INSTALL_DIR
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
cd weights
FILE="e2e_mask_rcnn_R_50_FPN_1x.pth"
if [ ! -f "$FILE" ]; then
    echo "Downloading MaskRCNN pretrained weights."
    wget https://download.pytorch.org/models/maskrcnn/e2e_mask_rcnn_R_50_FPN_1x.pth
fi
unset FILE
cp /home/user/trainer/training_files/trim_mask_rcnn.py trim_mask_rcnn.py
python trim_mask_rcnn.py
cd $INSTALL_DIR
# Create datasets folder if does not exist.
if [ ! -d "datasets" ]; then
    mkdir datasets
fi
