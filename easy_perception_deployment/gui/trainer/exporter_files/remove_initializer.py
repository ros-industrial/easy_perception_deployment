# MIT License
#
# Copyright (c) 2018 Facebook
# Copyright 2020 ROS-Industrial Consortium Asia Pacific
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse

import onnx


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, help='input model')
    parser.add_argument('--output', required=True, help='output model')
    args = parser.parse_args()
    return args


def remove_initializer_from_input():
    args = get_args()

    model = onnx.load(args.input)
    if model.ir_version < 4:
        print(
            'Model with ir_version below 4 requires \
            to include initilizer in graph input'
        )
        return

    inputs = model.graph.input
    name_to_input = {}
    for i in inputs:
        name_to_input[i.name] = i

    for initializer in model.graph.initializer:
        if initializer.name in name_to_input:
            inputs.remove(name_to_input[initializer.name])

    onnx.save(model, args.output)


if __name__ == '__main__':
    remove_initializer_from_input()
