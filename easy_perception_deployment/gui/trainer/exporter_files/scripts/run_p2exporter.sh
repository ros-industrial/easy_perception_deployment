#!/usr/bin/env bash

export START_DIR=$PWD

model_name=$1
timestamp=$2

eval "$(conda shell.bash hook)"
conda activate p2_exporter

cd $START_DIR/trainer/P2TrainFarm
cp trained_models/trained.pth p2_exporter/weights/custom/trained.pth
cd p2_exporter
python demo/export_to_onnx.py
python demo/remove_initializer.py --input weights/custom.onnx --output weights/exported_p2.onnx
cd $START_DIR/trainer/P2TrainFarm
cp p2_exporter/weights/exported_p2.onnx trained_models/$model_name-$timestamp.onnx

unset env_exists INSTALL_DIR START_DIR
