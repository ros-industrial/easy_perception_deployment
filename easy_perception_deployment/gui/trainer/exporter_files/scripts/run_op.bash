#!/usr/bin/env bash

cd /home/user/p3_exporter
python demo/export_to_onnx.py
python demo/remove_initializer.py --input weights/custom.onnx --output weights/exported_p3.onnx
cp --force weights/exported_p3.onnx /home/user/output.onnx

