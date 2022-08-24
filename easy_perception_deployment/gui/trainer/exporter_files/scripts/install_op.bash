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

# Function: Install dependencies within Export Docker Container.
# Static Analysis: shellcheck install_op.bash -x -e SC1091

EXPORT_MASKRCNN=$1

cd /home/user/ || exit
if $EXPORT_MASKRCNN ; then
    git clone https://github.com/cardboardcode/maskrcnn-benchmark.git --branch onnx_stage_mrcnn --single-branch p3_exporter --depth 1
    cd p3_exporter || exit
else
    git clone https://github.com/BowenBao/maskrcnn-benchmark.git --branch onnx_stage --single-branch p2_exporter --depth 1
    cd p2_exporter || exit
fi

export INSTALL_DIR=$PWD
git clone https://github.com/cardboardcode/cocoapi.git
cd cocoapi/PythonAPI || exit
python setup.py build_ext install
cd "$INSTALL_DIR" || exit
git clone https://github.com/cardboardcode/apex.git
cd apex || exit
python setup.py install
cd "$INSTALL_DIR" || exit
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

