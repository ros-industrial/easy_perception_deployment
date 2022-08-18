#!/usr/bin/env bash

EXPORT_MASKRCNN=$1

cd /home/user/
if $EXPORT_MASKRCNN ; then
    git clone https://github.com/cardboardcode/maskrcnn-benchmark.git --branch onnx_stage_mrcnn --single-branch p3_exporter --depth 1
    cd p3_exporter
else
    git clone https://github.com/BowenBao/maskrcnn-benchmark.git --branch onnx_stage --single-branch p2_exporter --depth 1
    cd p2_exporter
fi

export INSTALL_DIR=$PWD
git clone https://github.com/cardboardcode/cocoapi.git
cd cocoapi/PythonAPI
python setup.py build_ext install
cd $INSTALL_DIR
git clone https://github.com/cardboardcode/apex.git
cd apex
python setup.py install
cd $INSTALL_DIR
if $EXPORT_MASKRCNN ; then
    cp /home/user/trainer/exporter_files/modified_imports.py /home/user/p3_exporter/maskrcnn_benchmark/utils/imports.py
else
    cp /home/user/trainer/exporter_files/modified_imports.py /home/user/p2_exporter/maskrcnn_benchmark/utils/imports.py
fi
python setup.py build develop
if [ ! -d "configs/custom" ]; then
    mkdir -p configs/custom
fi
if [ ! -d "weights/custom" ]; then
    mkdir -p weights/custom
fi
cp /home/user/trainer/exporter_files/remove_initializer.py demo/remove_initializer.py
if $EXPORT_MASKRCNN ; then
    cp /home/user/trainer/exporter_files/export_to_p3_onnx.py demo/export_to_onnx.py
else
    cp /home/user/trainer/exporter_files/export_to_p2_onnx.py demo/export_to_onnx.py
fi

