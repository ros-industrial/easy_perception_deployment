#!/usr/bin/env bash

export START_DIR=$PWD

model_name=$1
timestamp=$2

eval "$(conda shell.bash hook)"
conda activate p3_exporter

cd $START_DIR/trainer/P3TrainFarm
cp trained_models/trained.pth p3_exporter/weights/custom/trained.pth
cd p3_exporter
python demo/export_to_onnx.py
python demo/remove_initializer.py --input weights/custom.onnx --output weights/exported_p3.onnx
cd $START_DIR/trainer/P3TrainFarm
cp p3_exporter/weights/exported_p3.onnx trained_models/$model_name-$timestamp.onnx

unset env_exists INSTALL_DIR START_DIR
