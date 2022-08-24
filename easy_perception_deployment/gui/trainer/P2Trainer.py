# Copyright 2022 ROS-Industrial Consortium Asia Pacific
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

from datetime import datetime

import os
import json
import yaml
import subprocess
import logging


class P2Trainer:
    '''
    The Precision-Level 2 (P2) Trainer class object instantiates a training
    session using PyTorch's MaskRCNN-Benchmark for taking an input dataset and
    fasterrcnn model from PyTorch model zoo in order to generate a
    custom-trained P2 ONNX model file.\n
    This model can then be deployed as a ROS2 package.
    '''
    def __init__(self, path_to_dataset, model_name,
                 label_list, max_iteration, checkpoint_period,
                 test_period, steps):
        '''
        The constructor.
        Sets all the required fixed path to various files needed to start
        a training session.\n
        Calls updateTrainingConfig function.
        '''
        self.p2_train_logger = logging.getLogger('p2_train')

        self.model_name = model_name
        self.label_list = label_list
        self.max_iteration = max_iteration
        self.checkpoint_period = checkpoint_period
        self.test_period = test_period
        self.steps = steps
        self._TRAIN_DOCKER_IMG = "cardboardcode/epd-trainer:latest"
        self._TRAIN_DOCKER_CONTAINER = "epd_p2_trainer"
        self._EXPORT_DOCKER_IMG = "cardboardcode/epd-exporter:latest"
        self._EXPORT_DOCKER_CONTAINER = "epd_p2_exporter"
        self.isGPUAvailableFlag = False

        self.path_to_dataset = path_to_dataset
        self.path_to_modif = (
            'trainer/training_files/modified_paths_catalog.py')
        self.path_to_training_config = (
            'trainer/training_files/fasterrcnn_training.yaml')
        self.path_to_export_config = (
            'trainer/exporter_files/fasterrcnn_export.yaml')
        self.path_to_trim_tools = (
            'trainer/training_files/trim_faster_rcnn.py')
        self.path_to_remove_init_tool = (
            'trainer/exporter_files/remove_initializer.py')
        self.path_to_export_modif = (
            'trainer/exporter_files/export_to_p2_onnx.py')

        self.updateTrainingConfig()
        self.updateTrainVerification()
        self.checkGPUAvailability()

    def updateTrainingConfig(self):
        '''
        A Mutator function that modifies the various training session config
        files.
        '''
        isCOCOFormat = False
        for label in self.label_list:
            if label == '__ignore__':
                isCOCOFormat = True
                break
        if isCOCOFormat:
            custom_class_no = len(self.label_list)
        else:
            custom_class_no = len(self.label_list) + 2

        # Load maskrcnn_training.yaml and
        # maskrcnn_export.yaml in order to
        # replace NUM_CLASSES.
        dict = {}
        with open(self.path_to_training_config) as file:
            dict = yaml.load(file, Loader=yaml.FullLoader)

        dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] = custom_class_no
        dict['SOLVER']['MAX_ITER'] = self.max_iteration
        dict['SOLVER']['CHECKPOINT_PERIOD'] = self.checkpoint_period
        dict['SOLVER']['TEST_PERIOD'] = self.test_period
        dict['SOLVER']['STEPS'] = self.steps

        with open(self.path_to_training_config, 'w') as file:
            documents = yaml.dump(dict, file)

        dict.clear()
        with open(self.path_to_export_config) as file:
            dict = yaml.load(file, Loader=yaml.FullLoader)

        dict['MODEL']['ROI_BOX_HEAD']['NUM_CLASSES'] = custom_class_no

        with open(self.path_to_export_config, 'w') as file:
            documents = yaml.dump(dict, file)

    def updateTrainVerification(self):
        _PATH_TO_P2_TRAIN_VERIFICATION = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_PATH_TO_P2_TRAIN_VERIFICATION)
        p2_train_verification = json.load(file)

        # Check if docker image cardboardcode/epd-trainer:latest exists
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=image",
            self._TRAIN_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()

        if self.docker_inspect_process.returncode != 0:
            p2_train_verification["isTrainFarmDockerImagePulled"] = False
        else:
            p2_train_verification["isTrainFarmDockerImagePulled"] = True

        # Check if docker container epd_p3_trainer exists
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._TRAIN_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()

        if self.docker_inspect_process.returncode != 0:
            p2_train_verification["isTrainFarmDockerContainerCreated"] = False
        else:
            p2_train_verification["isTrainFarmDockerContainerCreated"] = True

        # Check if docker image cardboardcode/epd-exporter:latest exists
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=image",
            self._EXPORT_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()

        if self.docker_inspect_process.returncode != 0:
            p2_train_verification["isExporterDockerImagePulled"] = False
        else:
            p2_train_verification["isExporterDockerImagePulled"] = True

        # Check if docker container epd_p2_trainer exists
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._EXPORT_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()

        if self.docker_inspect_process.returncode != 0:
            p2_train_verification["isExportDockerContainerCreated"] = False
        else:
            p2_train_verification["isExportDockerContainerCreated"] = True

        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_PATH_TO_P2_TRAIN_VERIFICATION, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def checkGPUAvailability(self):
        # Checks whether there is available GPU device.
        cmd = ["nvidia-smi"]
        inspect_gpu_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True)
        inspect_gpu_process.communicate()
        if inspect_gpu_process.returncode == 127:
            self.p2_train_logger.warn("[ nvidia-smi ] command not found. " +
                                      "Please install nvidia-driver.")
            self.isGPUAvailableFlag = False
        # Checks if CUDA has been installed.
        cmd = ["nvcc"]
        inspect_cuda_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True)
        inspect_cuda_process.communicate()
        if inspect_cuda_process.returncode == 127:
            self.p2_train_logger.warn("[ nvcc ] command not found. " +
                                      "Please install nvidia-driver.")
            self.isGPUAvailableFlag = False

        self.p2_train_logger.info("[ nvidia-smi ] command - FOUND")
        self.p2_train_logger.info("[ nvcc ] command - FOUND")
        self.isGPUAvailableFlag = True

    def train(self, debug):
        # If GPU is unavailable, output warning and exit.
        if not self.isGPUAvailableFlag:
            self.p2_train_logger.warning(
                "GPU not detected. Skipping [ train ]...")
            return None

        # Verify that P2TrainFarm has successfully set up before.
        # If P2TrainFarm has been set up.
        if not self.isTrainFarmSetupVerified():
            self.pullTrainFarmDockerImage()
            self.createTrainFarmDockerContainer()
            self.installTrainingDependencies()
        else:
            self.p2_train_logger.info(
                "P2 TrainFarm Setup - VERIFIED. Proceeding to train...")

        self.copyTrainingFiles()
        self.runTraining()

    def export(self, debug):
        # If GPU is unavailable, output warning and exit.
        if not self.isGPUAvailableFlag:
            self.p2_train_logger.warning(
                "GPU not detected. Skipping [ export ]...")
            return None

        # Verify that P2Exporter has successfully set up before.
        # If P2Exporter has been set up.
        if not self.isExporterSetupVerified():
            self.pullExporterDockerImage()
            self.createExportDockerContainer()
            self.installExporterDependencies()
        else:
            self.p2_train_logger.info("P2 Exporter Setup - VERIFIED. " +
                                      "Proceeding to export...")

        self.copyExportFiles()
        self.runExporter()

    def isTrainFarmSetupVerified(self):
        # Check p2_trainfarm_verification.json.
        # If one check is False, TrainFarm has not been built properly.
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)
        if p2_train_verification["isTrainFarmDockerImagePulled"] is False:
            return False
        if p2_train_verification["isTrainDependenciesInstalled"] is False:
            return False

        return True

    def isExporterSetupVerified(self):
        # Check p2_trainfarm_verification.json.
        # If one check is False, Exporter has not been built properly.
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)
        if p2_train_verification["isExporterDockerImagePulled"] is False:
            return False
        if p2_train_verification["isExportDependenciesInstalled"] is False:
            return False

        return True

    def pullTrainFarmDockerImage(self):
        # Check if docker image cardboardcode/epd-trainer:latest exists

        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=image",
            self._TRAIN_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker_inspect_process returns 0, image is missing.
        # Pull public docker image cardboardcode/epd-trainer:latest
        # Otherwise, proceed.
        docker_pull_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = ["sudo", "docker", "pull", self._TRAIN_DOCKER_IMG]
            self.docker_pull_process = subprocess.Popen(cmd)
            self.docker_pull_process.communicate()
            docker_pull_process_returncode = (
                self.docker_pull_process.returncode)
        else:
            self.p2_train_logger.info(
                self._TRAIN_DOCKER_IMG + " - Docker Image FOUND.")
            docker_pull_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if docker_pull_process_returncode == 0:
            p2_train_verification["isTrainFarmDockerImagePulled"] = True
        else:
            p2_train_verification["isTrainFarmDockerImagePulled"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def createTrainFarmDockerContainer(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._TRAIN_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker container missing,
        # Create docker container, epd_p3_trainer, from image.
        docker_construct_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = [
                "bash",
                "trainer/training_files/scripts/" +
                "create_trainfarm_docker_container.bash",
                "false"]
            self.docker_construct_process = subprocess.Popen(cmd)
            self.docker_construct_process.communicate()
            docker_construct_process_returncode = (
                self.docker_construct_process.returncode)
        else:
            self.p2_train_logger.info(
                self._TRAIN_DOCKER_CONTAINER + " - Docker Container FOUND.")
            docker_construct_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if docker_construct_process_returncode == 0:
            p2_train_verification["isTrainFarmDockerContainerCreated"] = True
        else:
            p2_train_verification["isTrainFarmDockerContainerCreated"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def pullExporterDockerImage(self):
        # Check if docker image cardboardcode/epd-exporter:latest exists

        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=image",
            self._EXPORT_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker_inspect_process returns 0, image is missing.
        # Pull public docker image cardboardcode/epd-exporter:latest
        # Otherwise, proceed.
        docker_pull_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = ["docker", "pull", self._EXPORT_DOCKER_IMG]
            self.docker_pull_process = subprocess.Popen(cmd)
            self.docker_pull_process.communicate()
            docker_pull_process_returncode = (
                self.docker_pull_process.returncode)
        else:
            self.p2_train_logger.info(
                self._EXPORT_DOCKER_IMG + " - Docker Image FOUND.")
            docker_pull_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if docker_pull_process_returncode == 0:
            p2_train_verification["isExporterDockerImagePulled"] = True
        else:
            p2_train_verification["isExporterDockerImagePulled"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def installTrainingDependencies(self):
        # Enter docker container
        # Install dependencies
        cmd = [
            "bash",
            "trainer/training_files/scripts/" +
            "prepare_trainfarm_docker_container.bash",
            "false"]
        self.install_depend_process = subprocess.Popen(cmd)
        self.install_depend_process.communicate()

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if self.install_depend_process.returncode == 0:
            p2_train_verification["isTrainDependenciesInstalled"] = True
        else:
            p2_train_verification["isTrainDependenciesInstalled"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def createExportDockerContainer(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._EXPORT_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker container missing,
        # Create docker container, epd_p3_trainer, from image.
        docker_construct_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = [
                "bash",
                "trainer/exporter_files/scripts/" +
                "create_exporter_docker_container.bash",
                "false"]
            self.docker_construct_process = subprocess.Popen(cmd)
            self.docker_construct_process.communicate()
            docker_construct_process_returncode = (
                self.docker_construct_process.returncode)
        else:
            self.p2_train_logger.info(self._EXPORT_DOCKER_CONTAINER +
                                      " - Docker Container FOUND.")
            docker_construct_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if docker_construct_process_returncode == 0:
            p2_train_verification["isExportDockerContainerCreated"] = True
        else:
            p2_train_verification["isExportDockerContainerCreated"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def installExporterDependencies(self):
        # Enter docker container
        # Install dependencies
        cmd = [
            "bash",
            "trainer/exporter_files/scripts/" +
            "prepare_exporter_docker_container.bash",
            "false"]
        self.install_depend_process = subprocess.Popen(cmd)
        self.install_depend_process.communicate()

        # If docker_pull_process succeeded,
        # Update p2_trainfarm_verification.json
        _path_to_p2_train_verification = "../config/p2_train_verification.json"
        # Load p2_trainfarm_verification.json
        file = open(_path_to_p2_train_verification)
        p2_train_verification = json.load(file)

        if self.install_depend_process.returncode == 0:
            p2_train_verification["isExportDependenciesInstalled"] = True
        else:
            p2_train_verification["isExportDependenciesInstalled"] = False
        json_object = json.dumps(p2_train_verification, indent=4)
        with open(_path_to_p2_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def copyTrainingFiles(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._TRAIN_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If missing, run installTrainingDependencies function.
        # Otherwise, run the training within Docker Container.
        if self.docker_inspect_process.returncode != 0:
            self.installTrainingDependencies()
        else:
            self.p2_train_logger.info(self._TRAIN_DOCKER_CONTAINER +
                                      " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/training_files/scripts/" +
            "copy_training_files.bash",
            "false"]
        self.copy_process = subprocess.Popen(cmd)
        self.copy_process.communicate()

    def runTraining(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._TRAIN_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If missing, run installTrainingDependencies function.
        # Otherwise, run the training within Docker Container.
        if self.docker_inspect_process.returncode != 0:
            self.installTrainingDependencies()
        else:
            self.p2_train_logger.info(self._TRAIN_DOCKER_CONTAINER +
                                      " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/training_files/scripts/run_training.bash",
            "false"]
        self.training_process = subprocess.Popen(cmd)
        self.training_process.communicate()

    def copyExportFiles(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._EXPORT_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If missing, run installExporterDependencies function.
        # Otherwise, run the export within Docker Container.
        if self.docker_inspect_process.returncode != 0:
            self.installExporterDependencies()
        else:
            self.p2_train_logger.info(self._EXPORT_DOCKER_CONTAINER +
                                      " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/exporter_files/scripts/copy_exporter_files.bash",
            "false"]
        self.copy_process = subprocess.Popen(cmd)
        self.copy_process.communicate()

    def runExporter(self):
        # Check if docker container exists.
        cmd = [
            "sudo",
            "docker",
            "inspect",
            "--type=container",
            self._EXPORT_DOCKER_CONTAINER]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If missing, run installExporterDependencies function.
        # Otherwise, run the export within Docker Container.
        if self.docker_inspect_process.returncode != 0:
            self.installExporterDependencies()
        else:
            self.p2_train_logger.info(self._EXPORT_DOCKER_CONTAINER +
                                      " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/exporter_files/scripts/run_exporter.bash",
            "false"]
        self.exporter_process = subprocess.Popen(cmd)
        self.exporter_process.communicate()

        # Rename and copy over generated output.onnx to EPD /data/model dir.
        _FILE_PATH_TO_ONNX_MODEL = "output.onnx"
        if os.path.exists("output.onnx"):
            timestamp = datetime.now()
            timestamp_string = timestamp.strftime("%d-%m-%Y-%H-%M-%S")
            TIMESTAMPED_ONNX_FILE_NAME = (
                self.model_name +
                "-" +
                timestamp_string +
                ".onnx")
            self.p2_train_logger.info(
                "ONNX model generated from .pth file. " +
                "File saved to [data/model/" +
                TIMESTAMPED_ONNX_FILE_NAME + "]...")
            cmd = ["cp",
                   _FILE_PATH_TO_ONNX_MODEL,
                   "../data/model/" + TIMESTAMPED_ONNX_FILE_NAME]
            self.shift_process = subprocess.Popen(cmd)
            self.shift_process.communicate()
        else:
            self.p2_train_logger.warning(
                "[ output.onnx ] - MISSING. " +
                "Something must have failed before this.")
