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

# Function: Run exporter and generate .onnx model file.
# Static Analysis: shellcheck run_op.bash -x -e SC1091

EXPORT_MASKRCNN=$1

if $EXPORT_MASKRCNN ; then
    cd /home/user/p3_exporter || exit
else
    cd /home/user/p2_exporter || exit
fi

python demo/export_to_onnx.py
python demo/remove_initializer.py --input weights/custom.onnx --output weights/exported.onnx
cp --force weights/exported.onnx /home/user/output.onnx

