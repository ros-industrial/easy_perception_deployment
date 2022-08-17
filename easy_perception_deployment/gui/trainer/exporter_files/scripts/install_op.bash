#!/usr/bin/env bash

cd /home/user/
git clone https://github.com/cardboardcode/maskrcnn-benchmark.git --branch onnx_stage_mrcnn --single-branch p3_exporter --depth 1
cd p3_exporter
export INSTALL_DIR=$PWD
git clone https://github.com/cardboardcode/cocoapi.git
cd cocoapi/PythonAPI
python setup.py build_ext install
cd $INSTALL_DIR
git clone https://github.com/cardboardcode/apex.git
cd apex
python setup.py install
cd $INSTALL_DIR
cp /home/user/trainer/exporter_files/modified_imports.py /home/user/p3_exporter/maskrcnn_benchmark/utils/imports.py
python setup.py build develop
if [ ! -d "configs/custom" ]; then
    mkdir -p configs/custom
fi
if [ ! -d "weights/custom" ]; then
    mkdir -p weights/custom
fi
cp /home/user/trainer/exporter_files/remove_initializer.py demo/remove_initializer.py
cp /home/user/trainer/exporter_files/export_to_p3_onnx.py demo/export_to_onnx.py

