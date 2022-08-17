#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

cd /home/user/maskrcnn-benchmark
if $TRAIN_MASKRCNN ; then
    python tools/train_net.py --config-file configs/custom/maskrcnn_training.yaml
else
    python tools/train_net.py --config-file configs/custom/fasterrcnn_training.yaml
fi

cp weights/custom/model_final.pth ../trained.pth
echo 'Generated saved weights in [ trained.pth ]. Please find under [ /home/user ].'
