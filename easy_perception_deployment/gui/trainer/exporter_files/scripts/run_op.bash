#!/usr/bin/env bash

EXPORT_MASKRCNN=$1

if $EXPORT_MASKRCNN ; then
    cd /home/user/p3_exporter
else
    cd /home/user/p2_exporter
fi

python demo/export_to_onnx.py
python demo/remove_initializer.py --input weights/custom.onnx --output weights/exported.onnx
cp --force weights/exported.onnx /home/user/output.onnx

