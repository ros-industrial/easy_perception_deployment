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
import sys
import json
import yaml
import subprocess

from trainer.P2Trainer import P2Trainer
from trainer.P3Trainer import P3Trainer

from windows.Train import TrainWindow

from datetime import date
from PySide2 import QtCore


def isGPUAvailable():
    # Checks whether there is available GPU device.
    cmd = ["nvidia-smi"]
    inspect_gpu_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True)
    inspect_gpu_process.communicate()
    if inspect_gpu_process.returncode == 127:
        print("[ nvidia-smi ] command not found. " +
              "Please install nvidia-driver.")
        return False
    # Checks if CUDA has been installed.
    cmd = ["nvcc"]
    inspect_cuda_process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True)
    inspect_cuda_process.communicate()
    if inspect_cuda_process.returncode == 127:
        print("[ nvcc ] command not found. Please install nvidia-driver.")
        return False

    print("[ nvidia-smi ] command - FOUND")
    print("[ nvcc ] command - FOUND")
    return True


# Check if nvidia-smi and nvcc has been installed correctly
# to verify Nvidia GPU is ready to be used.
if not isGPUAvailable():
    print("No GPU devices FOUND.")
    print("Please install nvidia-driver and CUDA. Exiting...")
    sys.exit(1)
else:
    print("GPU device FOUND. Proceeding...")

# Reset p2_train_verification.json and p3_train_verification
FILE_PATH_TO_P2_TRAIN_VERIFICATION = '../config/p2_train_verification.json'
FILE_PATH_TO_P3_TRAIN_VERIFICATION = '../config/p3_train_verification.json'
remove_1 = subprocess.Popen(['rm', FILE_PATH_TO_P2_TRAIN_VERIFICATION])
remove_1.communicate()
remove_2 = subprocess.Popen(['rm', FILE_PATH_TO_P3_TRAIN_VERIFICATION])
remove_2.communicate()

dict = {
    "isTrainFarmDockerImagePulled": False,
    "isTrainFarmDockerContainerCreated": False,
    "isTrainDependenciesInstalled": False,
    "isExporterDockerImagePulled": False,
    "isExportDockerContainerCreated": False,
    "isExportDependenciesInstalled": False
    }
json_object = json.dumps(dict, indent=4)
with open(FILE_PATH_TO_P2_TRAIN_VERIFICATION, 'w') as outfile:
    outfile.write(json_object)

with open(FILE_PATH_TO_P3_TRAIN_VERIFICATION, 'w') as outfile:
    outfile.write(json_object)

# Store path to test dataset as well as test label list.
if not os.path.exists("../test/custom_dataset"):
    print("[ test_dataset ] - MISSING. Folder has been " +
          "moved else where for unknown reason.")
    sys.exit()
else:
    PATH_TO_TEST_TRAIN_DATASET = os.path.abspath("../test/custom_dataset")


def test_P3Trainer_pullTrainFarmDockerImage(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_IMG = "cardboardcode/epd-trainer:latest"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.pullTrainFarmDockerImage()

    cmd = ["sudo", "docker", "inspect", "--type=image", _TRAIN_DOCKER_IMG]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P3Trainer_createTrainFarmDockerContainer(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p3_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.createTrainFarmDockerContainer()

    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _TRAIN_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P3Trainer_installTrainingDependencies(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p3_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.installTrainingDependencies()

    # Check if _TRAIN_DOCKER_CONTAINER Docker Container
    # has been successfully created.
    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _TRAIN_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0
    assert os.path.exists("p3_trainer") is True


def test_P3Trainer_copyTrainingFiles(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p3_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.copyTrainingFiles()

    assert os.path.exists("p3_trainer") is True
    # Check if custom_dataset has been transferred into p3_trainer
    assert os.path.exists("p3_trainer/datasets/custom_dataset") is True
    # Check if maskrcnn_training.yaml has been transferred into p3_trainer
    assert os.path.exists(
        "p3_trainer/configs/custom/maskrcnn_training.yaml") is True

    # Check that maskrcnn_training.yaml content are correct.
    dict = {}
    with open('p3_trainer/configs/custom/maskrcnn_training.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3
    assert dict['SOLVER']['MAX_ITER'] == 100
    assert dict['SOLVER']['CHECKPOINT_PERIOD'] == 100
    assert dict['SOLVER']['TEST_PERIOD'] == 100
    assert dict['SOLVER']['STEPS'] == '(100, 200, 300)'


def test_P3Trainer_runTraining(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.runTraining()

    # Check if trained.pth has been generated in root.
    assert os.path.exists("trained.pth") is True


def test_P3Trainer_pullExporterDockerImage(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _EXPORT_DOCKER_IMG = "cardboardcode/epd-exporter:latest"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.pullExporterDockerImage()

    cmd = ["sudo", "docker", "inspect", "--type=image", _EXPORT_DOCKER_IMG]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P3Trainer_createExportDockerContainer(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _EXPORT_DOCKER_CONTAINER = "epd_p3_exporter"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.createExportDockerContainer()

    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _EXPORT_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P3Trainer_installExporterDependencies(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _EXPORT_DOCKER_CONTAINER = "epd_p3_exporter"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.installExporterDependencies()

    # Check if _TRAIN_DOCKER_CONTAINER Docker Container
    # has been successfully created.
    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _EXPORT_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0
    assert os.path.exists("p3_exporter") is True


def test_P3Trainer_copyExportFiles(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p3_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.copyExportFiles()

    assert os.path.exists("p3_exporter") is True
    # Check if trained.pth has been transferred into p3_exporter
    assert os.path.exists("p3_exporter/weights/custom/trained.pth") is True
    # Check if maskrcnn_training.yaml has been transferred into p3_trainer
    assert os.path.exists(
        "p3_exporter/configs/custom/maskrcnn_export.yaml") is True

    # Check that maskrcnn_training.yaml content are correct.
    dict = {}
    with open('p3_exporter/configs/custom/maskrcnn_export.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3


def test_P3Trainer_runExporter(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'maskrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p3_trainer = P3Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p3_trainer.runExporter()

    # Check if trained.pth has been generated in root.
    assert os.path.exists("output.onnx") is True


def test_P2Trainer_pullTrainFarmDockerImage(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_IMG = "cardboardcode/epd-trainer:latest"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.pullTrainFarmDockerImage()

    cmd = ["sudo", "docker", "inspect", "--type=image", _TRAIN_DOCKER_IMG]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P2Trainer_createTrainFarmDockerContainer(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.createTrainFarmDockerContainer()

    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _TRAIN_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P2Trainer_installTrainingDependencies(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.installTrainingDependencies()

    # Check if _TRAIN_DOCKER_CONTAINER Docker Container
    # has been successfully created.
    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _TRAIN_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0
    assert os.path.exists("p2_trainer") is True


def test_P2Trainer_copyTrainingFiles(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.copyTrainingFiles()

    assert os.path.exists("p2_trainer") is True
    # Check if custom_dataset has been transferred into p3_trainer
    assert os.path.exists("p2_trainer/datasets/custom_dataset") is True
    # Check if maskrcnn_training.yaml has been transferred into p3_trainer
    assert os.path.exists(
        "p2_trainer/configs/custom/fasterrcnn_training.yaml") is True

    # Check that maskrcnn_training.yaml content are correct.
    dict = {}
    with open('p2_trainer/configs/custom/fasterrcnn_training.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3
    assert dict['SOLVER']['MAX_ITER'] == 100
    assert dict['SOLVER']['CHECKPOINT_PERIOD'] == 100
    assert dict['SOLVER']['TEST_PERIOD'] == 100
    assert dict['SOLVER']['STEPS'] == '(100, 200, 300)'


def test_P2Trainer_runTraining(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.runTraining()

    # Check if trained.pth has been generated in root.
    assert os.path.exists("trained.pth") is True


def test_P2Trainer_pullExporterDockerImage(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _EXPORT_DOCKER_IMG = "cardboardcode/epd-exporter:latest"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.pullExporterDockerImage()

    cmd = ["sudo", "docker", "inspect", "--type=image", _EXPORT_DOCKER_IMG]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P2Trainer_createExportDockerContainer(qtbot):

    path_to_dataset = 'path_to_dummy_dataset'
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.createExportDockerContainer()

    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _TRAIN_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0


def test_P2Trainer_installExporterDependencies(qtbot):
    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _EXPORT_DOCKER_CONTAINER = "epd_p2_exporter"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.installExporterDependencies()

    # Check if _TRAIN_DOCKER_CONTAINER Docker Container
    # has been successfully created.
    cmd = [
        "sudo",
        "docker",
        "inspect",
        "--type=container",
        _EXPORT_DOCKER_CONTAINER]

    docker_inspect_process = subprocess.Popen(
        cmd,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=None)
    docker_inspect_process.communicate()

    assert docker_inspect_process.returncode == 0
    assert os.path.exists("p2_exporter") is True


def test_P2Trainer_copyExportFiles(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']
    _TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.copyExportFiles()

    assert os.path.exists("p2_exporter") is True
    # Check if trained.pth has been transferred into p3_exporter
    assert os.path.exists("p2_exporter/weights/custom/trained.pth") is True
    # Check if maskrcnn_training.yaml has been transferred into p3_trainer
    assert os.path.exists(
        "p2_exporter/configs/custom/fasterrcnn_export.yaml") is True

    # Check that maskrcnn_training.yaml content are correct.
    dict = {}
    with open('p2_exporter/configs/custom/fasterrcnn_export.yaml') as file:
        dict = yaml.load(file, Loader=yaml.FullLoader)

    assert dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] == 3


def test_P2Trainer_runExporter(qtbot):

    global PATH_TO_TEST_TRAIN_DATASET
    path_to_dataset = PATH_TO_TEST_TRAIN_DATASET
    model_name = 'fasterrcnn'
    label_list = ['__ignore__', '_background_', 'teabox']

    widget = TrainWindow(True)
    qtbot.addWidget(widget)

    widget.max_iteration = 100
    widget.checkpoint_period = 100
    widget.test_period = 100
    widget.steps = '(100, 200, 300)'

    p2_trainer = P2Trainer(
        path_to_dataset,
        model_name,
        label_list,
        100,
        100,
        100,
        '(100, 200, 300)')

    p2_trainer.runExporter()

    # Check if trained.pth has been generated in root.
    assert os.path.exists("output.onnx") is True
