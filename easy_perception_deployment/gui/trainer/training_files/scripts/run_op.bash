#!/usr/bin/env bash

TRAIN_MASKRCNN=$1

if $TRAIN_MASKRCNN ; then
    cd /home/user/p3_trainer
else
    cd /home/user/p2_trainer
fi


if $TRAIN_MASKRCNN ; then
    python tools/train_net.py --config-file /home/user/p3_trainer/configs/custom/maskrcnn_training.yaml
else
    python tools/train_net.py --config-file /home/user/p2_trainer/configs/custom/fasterrcnn_training.yaml
fi

cp weights/custom/model_final.pth ../trained.pth
echo 'Generated saved weights in [ trained.pth ]. Please find under [ /home/user ].'
