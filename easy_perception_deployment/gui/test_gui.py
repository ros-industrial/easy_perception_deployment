# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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
import subprocess

from trainer.P1Trainer import P1Trainer
from trainer.P2Trainer import P2Trainer
from trainer.P3Trainer import P3Trainer

from windows.Counting import CountingWindow
from windows.Deploy import DeployWindow
from windows.Main import MainWindow
from windows.Train import TrainWindow

from datetime import date
from PySide2 import QtCore

# Clear all stored session_config.txt usecase_config.txt
if os.path.exists('../data/session_config.txt') and os.path.exists('../data/session_config.txt'):
    p1 = subprocess.Popen(['rm', '../data/session_config.txt'])
    p1.communicate()
    p2 = subprocess.Popen(['rm', '../data/usecase_config.txt'])
    p2.communicate()


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

    widget = DeployWindow()
    qtbot.addWidget(widget)

    if (os.path.exists('../data/session_config.txt') and
            os.path.exists('../data/session_config.txt')):
        p1 = subprocess.Popen(['rm', '../data/session_config.txt'])
        p1.communicate()
        p2 = subprocess.Popen(['rm', '../data/usecase_config.txt'])
        p2.communicate()

    assert widget._path_to_model == 'filepath/to/onnx/model'
    assert widget._path_to_label_list == 'filepath/to/classes/list/txt'
    assert widget.usecase_mode == 0


def test_invalidSession_invalidUseCase_DeployWindow(qtbot):

    test_session_config_content = ['test_filepath_to_model\n',
                                   'test_filepath_to_label_list\n',
                                   'visualize\n']
    # If session_config.txt is not present, create one.
    outF = open('../data/session_config.txt', 'w+')
    for line in test_session_config_content:
        outF.write(line)
    outF.close()

    test_usecase_config_content = ['-1\n']
    outF = open('../data/usecase_config.txt', 'w+')
    for line in test_usecase_config_content:
        outF.write(line)
    outF.close()

    widget = DeployWindow()
    qtbot.addWidget(widget)

    assert widget._path_to_model == 'filepath/to/onnx/model'
    assert widget._path_to_label_list == 'filepath/to/classes/list/txt'
    assert widget.usecase_mode == '-1'


def test_validSession_validUseCase_DeployWindow(qtbot):

    test_session_config_content = ['./data/model/squeezenet1.1-7.onnx\n',
                                   '/data/label_list/imagenet_classes.txt\n',
                                   'visualize\n',
                                   'CPU\n']
    outF = open('../data/session_config.txt', 'w+')
    for line in test_session_config_content:
        outF.write(line)
    outF.close()

    test_usecase_config_content = ['0\n']
    outF = open('../data/usecase_config.txt', 'w+')
    for line in test_usecase_config_content:
        outF.write(line)
    outF.close()

    widget = DeployWindow()
    qtbot.addWidget(widget)

    assert widget._path_to_model == './data/model/squeezenet1.1-7.onnx'
    assert widget._path_to_label_list == './data/label_list/imagenet_classes.txt'
    assert widget.usecase_mode == '0'


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


def test_setUseCase_Classification_DeployWindow(qtbot):

    widget = DeployWindow()
    qtbot.addWidget(widget)

    qtbot.keyClicks(widget.usecase_config_button, 'Counting')

    assert widget.counting_window.isVisible() is True


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

    usecase_config_lines = [line.rstrip('\n') for
                            line in open('../data/usecase_config.txt')]
    assert len(usecase_config_lines) == 1

    widget.setUseCase(color_matching_index)
    usecase_config_lines = [line.rstrip('\n') for
                            line in open('../data/usecase_config.txt')]
    assert len(usecase_config_lines) == 2


def test_setModel_DeployWindow(qtbot):

    widget = DeployWindow(True)
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.model_button, QtCore.Qt.LeftButton)

    assert widget._path_to_model == 'dummy_model_filepath'


def test_setLabelList_DeployWindow(qtbot):

    widget = DeployWindow(True)
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.list_button, QtCore.Qt.LeftButton)

    assert widget._path_to_label_list == 'dummy_label_list_filepath'


def test_setP1_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.p1_button, QtCore.Qt.LeftButton)

    assert widget._precision_level == 1


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

    qtbot.mouseClick(widget.p1_button, QtCore.Qt.LeftButton)
    qtbot.keyClicks(widget.model_selector, 'resnet')

    assert widget.model_name == 'resnet'

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

    widget._precision_level = 1
    widget._path_to_dataset = 'invalid_path_to_dataset'
    qtbot.mouseClick(widget.validate_button, QtCore.Qt.LeftButton)

    assert widget._is_dataset_labelled is False

    widget._precision_level = 2
    qtbot.mouseClick(widget.validate_button, QtCore.Qt.LeftButton)
    assert widget._is_dataset_labelled is False

    widget._precision_level = 3
    qtbot.mouseClick(widget.validate_button, QtCore.Qt.LeftButton)
    assert widget._is_dataset_labelled is False


def test_populateModelSelector_TrainWindow(qtbot):

    widget = TrainWindow()
    qtbot.addWidget(widget)

    widget._precision_level = 1
    widget.populateModelSelector()

    assert len(widget._model_list) == 7

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


def test_setDataset_TrainWindow(qtbot):

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.dataset_button, QtCore.Qt.LeftButton)

    assert widget._is_dataset_linked is True


def test_conformDatasetToCOCO_TrainWindow(qtbot):

    if not os.path.exists('../data/datasets/p2p3_dummy_dataset'):
        p1 = subprocess.Popen(['mkdir', '-p', '../data/datasets/p2p3_dummy_dataset/train_dataset'])
        p1.communicate()
        p2 = subprocess.Popen(['mkdir', '-p', '../data/datasets/p2p3_dummy_dataset/val_dataset'])
        p2.communicate()

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    qtbot.mouseClick(widget.generate_button, QtCore.Qt.LeftButton)

    assert widget.label_train_process is not None
    widget.label_train_process.kill()
    assert widget.label_val_process is not None
    widget.label_val_process.kill()

    # Clean up test materials.
    if os.path.exists('../data/datasets/p2p3_dummy_dataset'):
        p3 = subprocess.Popen(['rm', '-r', '../data/datasets/p2p3_dummy_dataset'])
        p3.communicate()


def test_closeWindow_CountingWindow(qtbot):

    path_to_labellist = '../data/label_list/coco_classes.txt'
    path_to_usecase_config = '../data/usecase_config.txt'
    widget = CountingWindow(path_to_labellist, path_to_usecase_config)
    qtbot.addWidget(widget)

    widget.show()
    qtbot.mouseClick(widget.cancel_button, QtCore.Qt.LeftButton)
    assert widget.isVisible() is False


def test_writeToUseCaseConfig_CountingWindow(qtbot):

    path_to_labellist = '../data/label_list/coco_classes.txt'
    path_to_usecase_config = '../data/usecase_config.txt'
    widget = CountingWindow(path_to_labellist, path_to_usecase_config)
    qtbot.addWidget(widget)

    widget.show()

    widget._select_list = ['person']
    qtbot.mouseClick(widget.finish_button, QtCore.Qt.LeftButton)

    select_list = [line.rstrip('\n') for
                   line in open('../data/usecase_config.txt')]
    assert select_list[0] == '1'
    assert select_list[1] == 'person'
    assert widget.isVisible() is False


def test_addObject_CountingWindow():

    path_to_labellist = '../data/label_list/coco_classes.txt'
    path_to_usecase_config = '../data/usecase_config.txt'
    widget = CountingWindow(path_to_labellist, path_to_usecase_config)

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


def test_P1Trainer():

    if not os.path.exists('../data/datasets/hymenoptera_data'):
        p1 = subprocess.Popen(['wget',
                               'https://download.pytorch.org/tutorial/hymenoptera_data.zip',
                               '--directory-prefix=../data/datasets/'])
        p1.communicate()
        p2 = subprocess.Popen(['unzip',
                               '../data/datasets/hymenoptera_data.zip',
                               '-d',
                               '../data/datasets/'])
        p2.communicate()

    path_to_dataset = '../data/datasets/hymenoptera_data'
    model_name = 'inception'
    label_list = ['ants', 'bees']

    p1_trainer = P1Trainer(path_to_dataset, model_name, label_list)

    p1_trainer.train(True)

    output_pth_filename = './trainer/P1TrainFarm/' + model_name + '_' + str(date.today()) + '.pth'
    output_model_filename = '../data/model/' + model_name + '_' + str(date.today()) + '.onnx'
    assert os.path.exists(output_pth_filename) is True
    assert os.path.exists(output_model_filename) is True
    p1_model_array = ['alexnet', 'vgg', 'squeezenet', 'densenet', 'resnet', 'mobilenet']
    for model in p1_model_array:
        model_ft, input_size = p1_trainer.initialize_model(model,
                                                           len(label_list),
                                                           True)
        assert model_ft is not None

    model_ft, input_size = p1_trainer.initialize_model('invalid_model',
                                                       len(label_list),
                                                       True)
    assert model_ft is None

    # Clean up test materials.
    if (os.path.exists('../data/datasets/hymenoptera_data') and
            os.path.exists('../data/datasets/hymenoptera_data.zip') and
            os.path.exists('./trainer/P1TrainFarm')):
        p3 = subprocess.Popen(['rm',
                               '-r',
                               '../data/datasets/hymenoptera_data',
                               '../data/datasets/hymenoptera_data.zip',
                               './trainer/P1TrainFarm',
                               output_model_filename])
        p3.communicate()


def test_P2Trainer():

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    path_label_list = '../data/label_list/coco_classes.txt'

    if os.path.exists(path_label_list):
        label_list = [line.rstrip('\n') for line in open(path_label_list)]
    else:
        label_list = ['ant', 'bees']

    p2_trainer = P2Trainer(path_to_dataset, model_name, label_list)

    p2_trainer.train(True)
    assert p2_trainer.create_process is not None
    p2_trainer.create_process.kill()
    assert p2_trainer.run_process is not None
    p2_trainer.run_process.kill()
    assert p2_trainer.build_export_process is not None
    p2_trainer.build_export_process.kill()
    assert p2_trainer.export_process is not None
    p2_trainer.export_process.kill()

    if os.path.exists('./trainer/P2TrainFarm'):
        p1 = subprocess.Popen(['rm', '-r', './trainer/P2TrainFarm'])
        p1.communicate()


def test_P3Trainer():

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    path_label_list = '../data/label_list/coco_classes.txt'

    if os.path.exists(path_label_list):
        label_list = [line.rstrip('\n') for line in open(path_label_list)]
    else:
        label_list = ['ant', 'bees']

    p3_trainer = P3Trainer(path_to_dataset, model_name, label_list)

    p3_trainer.train(True)
    assert p3_trainer.create_process is not None
    p3_trainer.create_process.kill()
    assert p3_trainer.run_process is not None
    p3_trainer.run_process.kill()
    assert p3_trainer.build_export_process is not None
    p3_trainer.build_export_process.kill()
    assert p3_trainer.export_process is not None
    p3_trainer.export_process.kill()

    if os.path.exists('./trainer/P3TrainFarm'):
        p1 = subprocess.Popen(['rm', '-r', './trainer/P3TrainFarm'])
        p1.communicate()
