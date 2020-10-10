# Copyright 2020 ROS-Industrial Consortium Asia Pacific
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

import torch
keys_to_remove = [
    'module.roi_heads.box.predictor.cls_score.weight',
    'module.roi_heads.box.predictor.cls_score.bias',
    'module.roi_heads.box.predictor.bbox_pred.weight',
    'module.roi_heads.box.predictor.bbox_pred.bias',

]


def trim_maskrcnn_benchmark_model(model_path: str, trimmed_model_path: str):
    state_dict = torch.load(model_path, map_location='cpu')

    model = state_dict['model']

    for key in keys_to_remove:
        if key in model:
            del model[key]
            print('key: {} is removed'.format(key))
        else:
            print('key: {} is not present'.format(key))

    print('Also deleting optimizer, scheduler, and iteration entries')
    del state_dict['optimizer']
    del state_dict['scheduler']
    del state_dict['iteration']

    torch.save(state_dict, trimmed_model_path)
    print(f'saved to: {trimmed_model_path}')


model_path = 'e2e_faster_rcnn_R_50_FPN_1x.pth'
trimmed_model_path = 'e2e_faster_rcnn_R_50_FPN_1x_trimmed.pth'
trim_maskrcnn_benchmark_model(model_path, trimmed_model_path)
