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

import os
import json
import yaml
import subprocess

from trainer.P2Trainer import P2Trainer
from trainer.P3Trainer import P3Trainer

from windows.Counting import CountingWindow
from windows.Deploy import DeployWindow
from windows.Main import MainWindow
from windows.Train import TrainWindow

from datetime import date
from PySide2 import QtCore

# Clear all stored session_config.json usecase_config.json
if (os.path.exists('../config/session_config.json') and
   os.path.exists('../config/usecase_config.json')):
    p1 = subprocess.Popen(['rm', '../config/session_config.json'])
    p1.communicate()
    p2 = subprocess.Popen(['rm', '../config/usecase_config.json'])
    p2.communicate()

    dict = {
        "path_to_model": './data/model/MaskRCNN-10.onnx',
        "path_to_label_list": './data/label_list/coco_classes.txt',
        "visualizeFlag": 'visualize',
        "useCPU": 'CPU'
        }
    json_object = json.dumps(dict, indent=4)
    with open('../config/session_config.json', 'w') as outfile:
        outfile.write(json_object)

    dict = {"usecase_mode": 0}
    json_object = json.dumps(dict, indent=4)
    with open('../config/usecase_config.json', 'w') as outfile:
        outfile.write(json_object)


def test_init_MainWindow(qtbot):

    widget = MainWindow()
    qtbot.addWidget(widget)

    widget.show()

    assert widget.isVisible() is True


def test_openTrain_MainWindow(qtbot):

    widget = MainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.train_button, QtCore.Qt.LeftButton)

    assert widget.train_window.isVisible() is True
    assert widget.isTrainOpen is True

    qtbot.mouseClick(widget.train_button, QtCore.Qt.LeftButton)

    assert widget.train_window.isVisible() is False
    assert widget.isTrainOpen is False


def test_openDeploy_MainWindow(qtbot):

    widget = MainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.deploy_button, QtCore.Qt.LeftButton)

    assert widget.deploy_window.isVisible() is True
    assert widget.isDeployOpen is True

    qtbot.mouseClick(widget.deploy_button, QtCore.Qt.LeftButton)

    assert widget.deploy_window.isVisible() is False
    assert widget.isDeployOpen is False


def test_closeAll_MainWindow(qtbot):

    widget = MainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.quit_button, QtCore.Qt.LeftButton)

    assert widget.isVisible() is False
    assert widget.train_window.isVisible() is False
    assert widget.deploy_window.isVisible() is False


def test_emptySession_emptyUseCase_DeployWindow(qtbot):

    if (os.path.exists('../config/session_config.json') and
            os.path.exists('../config/usecase_config.json')):
        p1 = subprocess.Popen(['rm', '../config/session_config.json'])
        p1.communicate()
        p2 = subprocess.Popen(['rm', '../config/usecase_config.json'])
        p2.communicate()

    widget = DeployWindow()
    qtbot.addWidget(widget)

    assert widget._path_to_model == 'filepath/to/onnx/model'
    assert widget._path_to_label_list == 'filepath/to/classes/list/txt'
    assert widget.usecase_mode == 0


def test_invalidSession_invalidUseCase_DeployWindow(qtbot):

    dict = {
        "path_to_model": 'test_filepath_to_model',
        "path_to_label_list": 'test_filepath_to_label_list',
        "visualizeFlag": 'visualize',
        "throwCPU": 'CPU'
        }
    json_object = json.dumps(dict, indent=4)
    with open('../config/session_config.json', 'w') as outfile:
        outfile.write(json_object)

    dict = {"usecase_mode": -1}
    json_object = json.dumps(dict, indent=4)
    with open('../config/usecase_config.json', 'w') as outfile:
        outfile.write(json_object)

    widget = DeployWindow()
    qtbot.addWidget(widget)

    assert widget._path_to_model == 'filepath/to/onnx/model'
    assert widget._path_to_label_list == 'filepath/to/classes/list/txt'
    assert widget.usecase_mode == 0


def test_validSession_validUseCase_DeployWindow(qtbot):

    local_path_to_model = './data/model/MaskRCNN-10.onnx'
    local_path_to_label_list = ('./data/label_list/' +
                                'coco_classes.txt')

    dict = {
        "path_to_model": local_path_to_model,
        "path_to_label_list": local_path_to_label_list,
        "visualizeFlag": 'visualize',
        "useCPU": 'CPU'
        }
    json_object = json.dumps(dict, indent=4)
    with open('../config/session_config.json', 'w') as outfile:
        outfile.write(json_object)

    dict = {"usecase_mode": 0}
    json_object = json.dumps(dict, indent=4)
    with open('../config/usecase_config.json', 'w') as outfile:
        outfile.write(json_object)

    widget = DeployWindow()
    qtbot.addWidget(widget)

    assert widget._path_to_model == local_path_to_model
    assert widget._path_to_label_list == local_path_to_label_list
    assert widget.usecase_mode == 0


def test_deployPackage_DeployWindow(qtbot):

    widget = DeployWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.run_button, QtCore.Qt.LeftButton)

    isDeployScriptRunning = widget._deploy_process.poll()
    assert isDeployScriptRunning is None
    assert widget._is_running is True

    widget._deploy_process.kill()

    qtbot.mouseClick(widget.run_button, QtCore.Qt.LeftButton)

    isKillScriptRunning = widget._kill_process.poll()
    assert isKillScriptRunning is None
    assert widget._is_running is False


def test_setUseCase_DeployWindow(qtbot):

    widget = DeployWindow(True)
    qtbot.addWidget(widget)

    classification_index = 0
    color_matching_index = 0

    for i in range(0, len(widget.usecase_list)):
        if widget.usecase_list[i] == 'Classification':
            classification_index = i
        if widget.usecase_list[i] == 'Color-Matching':
            color_matching_index = i

    widget.setUseCase(classification_index)
    f = open('../config/usecase_config.json')
    data = json.load(f)
    usecase_mode = data['usecase_mode']
    assert usecase_mode == 0

    widget.setUseCase(color_matching_index)
    f = open('../config/usecase_config.json')
    data = json.load(f)
    usecase_mode = data['usecase_mode']
    assert usecase_mode == 2


def test_setP2_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.p2_button, QtCore.Qt.LeftButton)

    assert widget._precision_level == 2


def test_setP3_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.p3_button, QtCore.Qt.LeftButton)

    assert widget._precision_level == 3


def test_setModel_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.p2_button, QtCore.Qt.LeftButton)
    qtbot.keyClicks(widget.model_selector, 'fasterrcnn')

    assert widget.model_name == 'fasterrcnn'

    qtbot.mouseClick(widget.p3_button, QtCore.Qt.LeftButton)
    qtbot.keyClicks(widget.model_selector, 'maskrcnn')

    assert widget.model_name == 'maskrcnn'


def test_runLabelme_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.label_button, QtCore.Qt.LeftButton)

    isLabelmeRunning = widget.label_process.poll()
    assert isLabelmeRunning is None

    widget.label_process.kill()


def test_validateTraining_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    widget._precision_level = 2

    widget.validateTraining()
    assert widget.buttonConnected is False

    widget._is_model_ready = True
    widget.validateTraining()
    assert widget.buttonConnected is False

    widget._is_dataset_linked = True
    widget.validateTraining()
    assert widget.buttonConnected is False

    widget._is_labellist_linked = True
    widget.validateTraining()
    assert widget.buttonConnected is False

    widget._precision_level = 1
    widget.validateTraining()
    assert widget.buttonConnected is False

    widget.buttonConnected = False
    widget._precision_level = 2
    widget._is_dataset_labelled = True
    widget.validateTraining()
    assert widget.buttonConnected is True


def test_validateDataset_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    widget._precision_level = 2
    widget._path_to_dataset = 'invalid_path_to_dataset'
    qtbot.mouseClick(widget.validate_button, QtCore.Qt.LeftButton)
    assert widget._is_dataset_labelled is False

    widget._precision_level = 3
    qtbot.mouseClick(widget.validate_button, QtCore.Qt.LeftButton)
    assert widget._is_dataset_labelled is False


def test_populateModelSelector_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    widget._precision_level = 2
    widget.populateModelSelector()

    assert widget._model_list[0] == 'fasterrcnn'

    widget._precision_level = 3
    widget.populateModelSelector()

    assert widget._model_list[0] == 'maskrcnn'


def test_checkModelReady_TrainWindow():

    widget = TrainWindow(True)

    widget.setModel(1)

    assert widget._is_model_ready is True


def test_setLabelList_TrainWindow(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.list_button, QtCore.Qt.LeftButton)

    assert widget._is_labellist_linked is True


def test_setMax_Iteration(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 1000

    qtbot.mouseClick(widget.maxiter_button, QtCore.Qt.LeftButton)

    assert widget.max_iteration == 1000


def test_setCheckPoint_Period(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.checkpoint_period = 100

    qtbot.mouseClick(widget.checkpointp_button, QtCore.Qt.LeftButton)

    assert widget.checkpoint_period == 100


def test_setTest_Period(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.test_period = 100

    qtbot.mouseClick(widget.testp_button, QtCore.Qt.LeftButton)

    assert widget.test_period == 100


def test_setSteps_Period(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.steps = '(100, 200, 300)'

    qtbot.mouseClick(widget.steps_button, QtCore.Qt.LeftButton)

    assert widget.steps == '(100, 200, 300)'


# def test_conformDatasetToCOCO_TrainWindow(qtbot):


def test_writeToUseCaseConfig_CountingWindow(qtbot):

    path_to_labellist = './data/label_list/coco_classes.txt'
    path_to_usecase_config = '../config/usecase_config.json'
    widget = CountingWindow(path_to_labellist, path_to_usecase_config)
    qtbot.addWidget(widget)

    widget.show()

    widget._select_list = ['person']
    qtbot.mouseClick(widget.finish_button, QtCore.Qt.LeftButton)

    f = open(path_to_usecase_config)
    data = json.load(f)
    usecase_mode = data['usecase_mode']
    class_list = data['class_list']

    assert usecase_mode == 1
    assert class_list[0] == 'person'
    assert widget.isVisible() is False


def test_addObject_CountingWindow():

    path_to_label_list = './data/label_list/coco_classes.txt'
    path_to_usecase_config = './config/usecase_config.json'
    widget = CountingWindow(path_to_label_list, path_to_usecase_config)

    widget.addObject(0)
    assert len(widget._select_list) == 1

    widget.addObject(0)
    assert len(widget._select_list) == 1

    widget.addObject(1)
    assert len(widget._select_list) == 2

    widget.removeObject(0)
    widget.removeObject(0)
    assert len(widget._select_list) == 0

    widget.removeObject(0)
    assert len(widget._select_list) == 0


def test_closeWindow_CountingWindow(qtbot):

    path_to_labellist = './data/label_list/coco_classes.txt'
    path_to_usecase_config = './config/usecase_config.json'
    widget = CountingWindow(path_to_labellist, path_to_usecase_config)
    qtbot.addWidget(widget)

    widget.show()
    qtbot.mouseClick(widget.cancel_button, QtCore.Qt.LeftButton)
    assert widget.isVisible() is False


def test_P2Trainer_Training_Config(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'test_object']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 1000
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        1000,
        100,
        100,
        '(100, 200, 300)')

    dict = {}
    with open('trainer/training_files/fasterrcnn_training.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3
    assert dict['SOLVER']['MAX_ITER'] == 1000
    assert dict['SOLVER']['CHECKPOINT_PERIOD'] == 100
    assert dict['SOLVER']['TEST_PERIOD'] == 100
    assert dict['SOLVER']['STEPS'] == '(100, 200, 300)'


def test_P3Trainer_Training_Config(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'test_object']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 1000
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        1000,
        100,
        100,
        '(100, 200, 300)')

    dict = {}
    with open('trainer/training_files/maskrcnn_training.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3
    assert dict['SOLVER']['MAX_ITER'] == 1000
    assert dict['SOLVER']['CHECKPOINT_PERIOD'] == 100
    assert dict['SOLVER']['TEST_PERIOD'] == 100
    assert dict['SOLVER']['STEPS'] == '(100, 200, 300)'
