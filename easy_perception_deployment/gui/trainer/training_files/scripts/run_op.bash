#!/usr/bin/env bash

# Copyright 2022 Advanced Remanufacturing and Technology Centre
# Copyright 2022 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Function: Run training.
# Static Analysis: shellcheck run_op.bash -x -e SC1091

TRAIN_MASKRCNN=$1

if $TRAIN_MASKRCNN ; then
    cd /home/user/p3_trainer || exit
else
    cd /home/user/p2_trainer || exit
fi


if $TRAIN_MASKRCNN ; then
    python tools/train_net.py --config-file /home/user/p3_trainer/configs/custom/maskrcnn_training.yaml
else
    python tools/train_net.py --config-file /home/user/p2_trainer/configs/custom/fasterrcnn_training.yaml
fi

cp weights/custom/model_final.pth ../trained.pth
echo 'Generated saved weights in [ trained.pth ]. Please find under [ /home/user ].'
