#!/usr/bin/env bash

cd /home/user/maskrcnn-benchmark
python tools/train_net.py --config-file configs/custom/maskrcnn_training.yaml
cp weights/custom/model_final.pth ../trained.pth
echo 'Generated saved weights in [ trained.pth ]. Please find under [ /home/user ].'
