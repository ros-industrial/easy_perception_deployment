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

from io import BytesIO

import os

from maskrcnn_benchmark.config import cfg
from maskrcnn_benchmark.structures.image_list import ImageList

import numpy

from PIL import Image

from predictor import COCODemo

import pytorch_export_patch

import requests

import torch

if __name__ == '__main__':
    # load config from file and command-line arguments
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg.merge_from_file(
        os.path.join(project_dir,
                     'configs/custom/fasterrcnn_export.yaml'))
    cfg.merge_from_list(['MODEL.DEVICE', 'cpu'])
    cfg.freeze()

    # prepare object that handles inference plus adds predictions on top of image
    coco_demo = COCODemo(
        cfg,
        confidence_threshold=0.7,
        show_mask_heatmaps=False,
        masks_per_dim=2,
        min_image_size=480,
    )

    # prepare for onnx export
    coco_demo.model.prepare_onnx_export()


def single_image_to_top_predictions(image):
    image_list = ImageList(image.unsqueeze(0), [(int(image.size(-2)), int(image.size(-1)))])

    for param in coco_demo.model.parameters():
        param.requires_grad = False

    result, = coco_demo.model(image_list)
    scores = result.get_field('scores')
    result = (result.bbox, result.get_field('labels'), scores)
    return result


class FRCNNModel(torch.nn.Module):

    def forward(self, image):
        return single_image_to_top_predictions(image)


def fetch_image(url):
    response = requests.get(url)
    return Image.open(BytesIO(response.content)).convert('RGB')


if __name__ == '__main__':
    pil_image = fetch_image(
        url='http://farm3.staticflickr.com/' +
            '2469/3915380994_2e611b1779_z.jpg')

    """
    Preprocessing image.
    """
    # convert to BGR format
    image = torch.from_numpy(numpy.array(pil_image)[:, :, [2, 1, 0]])
    original_image = image
    image = torch.nn.functional.upsample(image.permute(2, 0, 1)
                                         .unsqueeze(0)
                                         .to(torch.float), size=(960, 1280))
    image = image.to(torch.uint8).squeeze(0).permute(1, 2, 0).to('cpu')

    image = image.permute(2, 0, 1)

    if not cfg.INPUT.TO_BGR255:
        image = image.float() / 255.0
        image = image[[2, 1, 0]]
    else:
        image = image.float()

    # we absolutely want fixed size (int) here (or we run into a tracing error (or bug?)
    # or we might later decide to make things work with variable size...
    image = image - torch.tensor(cfg.INPUT.PIXEL_MEAN)[:, None, None].to('cpu')

    model_path = 'weights/custom.onnx'

    with torch.no_grad():
        model = FRCNNModel()
        model.eval()

        torch.onnx.export(model,
                          (image,),
                          model_path,
                          verbose=True,
                          opset_version=10,
                          strip_doc_string=True,
                          do_constant_folding=True)

        pytorch_export_patch.postprocess_model(model_path)
