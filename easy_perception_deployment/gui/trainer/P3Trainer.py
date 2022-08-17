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


class P3Trainer:
    '''
    The Precision-Level 3 (P3) Trainer class object instantiates a training
    session using PyTorch's MaskRCNN-Benchmark for taking an input dataset and
    maskrcnn model from PyTorch model zoo in order to generate a
    custom-trained P3 ONNX model file.\n
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
        self.model_name = model_name
        self.label_list = label_list
        self.max_iteration = max_iteration
        self.checkpoint_period = checkpoint_period
        self.test_period = test_period
        self.steps = steps
        self._TRAIN_DOCKER_IMG = "cardboardcode/epd-p3-trainfarm:latest"
        self._TRAIN_DOCKER_CONTAINER = "epd_p3_trainer"
        self._EXPORT_DOCKER_IMG = "cardboardcode/epd-p3-exporter:latest"
        self._EXPORT_DOCKER_CONTAINER = "epd_p3_exporter"

        self.create_process = None
        self.run_process = None
        self.build_export_process = None
        self.export_process = None

        self.path_to_dataset = path_to_dataset
        self.path_to_modif = (
            'trainer/training_files/modified_paths_catalog.py')
        self.path_to_training_config = (
            'trainer/training_files/maskrcnn_training.yaml')
        self.path_to_export_config = (
            'trainer/exporter_files/maskrcnn_export.yaml')
        self.path_to_trim_tools = (
            'trainer/training_files/trim_mask_rcnn.py')
        self.path_to_remove_init_tool = (
            'trainer/exporter_files/remove_initializer.py')
        self.path_to_export_modif = (
            'trainer/exporter_files/export_to_p3_onnx.py')

        self.updateTrainingConfig()

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

    def train(self, debug):
        # Verify that P3TrainFarm has successfully set up before.
        # If P3TrainFarm has been set up.
        if not self.isTrainFarmSetupVerified():
            self.pullTrainFarmDockerImage()
            self.installTrainingDependencies()
        else:
            print("P3 TrainFarm Setup - VERIFIED. Proceeding to train...")

        self.copyTrainingFiles()
        self.runTraining()

    def export(self, debug):
        # Verify that P3Exporter has successfully set up before.
        # If P3Exporter has been set up.
        if not self.isExporterSetupVerified():
            self.pullExporterDockerImage()
            self.installExporterDependencies()
        else:
            print("P3 Exporter Setup - VERIFIED. Proceeding to export...")

        self.copyExportFiles()
        self.runExporter()

    def isTrainFarmSetupVerified(self):
        # Check p3_trainfarm_verification.json.
        # If one check is False, TrainFarm has not been built properly.
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)
        if p3_train_verification["isTrainFarmDockerImagePulled"] is False:
            return False
        if p3_train_verification["isTrainDependenciesInstalled"] is False:
            return False

        return True

    def isExporterSetupVerified(self):
        # Check p3_trainfarm_verification.json.
        # If one check is False, Exporter has not been built properly.
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)
        if p3_train_verification["isExporterDockerImagePulled"] is False:
            return False
        if p3_train_verification["isExportDependenciesInstalled"] is False:
            return False

        return True

    def pullTrainFarmDockerImage(self):
        # Check if docker image cardboardcode/epd-p3-trainfarm:latest exists

        cmd = ["docker", "inspect", "--type=image", self._TRAIN_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker_inspect_process returns 0, image is missing.
        # Pull public docker image cardboardcode/epd-p3-trainfarm:latest
        # Otherwise, proceed.
        docker_pull_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = ["docker", "pull", self._TRAIN_DOCKER_IMG]
            self.docker_pull_process = subprocess.Popen(cmd)
            self.docker_pull_process.communicate()
            docker_pull_process_returncode = (
                self.docker_pull_process.returncode)
        else:
            print(self._TRAIN_DOCKER_IMG + " - Docker Image FOUND.")
            docker_pull_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p3_trainfarm_verification.json
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)

        if docker_pull_process_returncode == 0:
            p3_train_verification["isTrainFarmDockerImagePulled"] = True
        else:
            p3_train_verification["isTrainFarmDockerImagePulled"] = False
        json_object = json.dumps(p3_train_verification, indent=4)
        with open(_path_to_p3_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def pullExporterDockerImage(self):
        # Check if docker image cardboardcode/epd-p3-exporter:latest exists

        cmd = ["docker", "inspect", "--type=image", self._EXPORT_DOCKER_IMG]

        self.docker_inspect_process = subprocess.Popen(
            cmd,
            universal_newlines=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=None)
        self.docker_inspect_process.communicate()
        # If docker_inspect_process returns 0, image is missing.
        # Pull public docker image cardboardcode/epd-p3-exporter:latest
        # Otherwise, proceed.
        docker_pull_process_returncode = None
        if self.docker_inspect_process.returncode != 0:
            cmd = ["docker", "pull", self._EXPORT_DOCKER_IMG]
            self.docker_pull_process = subprocess.Popen(cmd)
            self.docker_pull_process.communicate()
            docker_pull_process_returncode = (
                self.docker_pull_process.returncode)
        else:
            print(self._EXPORT_DOCKER_IMG + " - Docker Image FOUND.")
            docker_pull_process_returncode = 0

        # If docker_pull_process succeeded,
        # Update p3_trainfarm_verification.json
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)

        if docker_pull_process_returncode == 0:
            p3_train_verification["isExporterDockerImagePulled"] = True
        else:
            p3_train_verification["isExporterDockerImagePulled"] = False
        json_object = json.dumps(p3_train_verification, indent=4)
        with open(_path_to_p3_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def installTrainingDependencies(self):
        # Check if docker container exists.
        cmd = [
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
        if self.docker_inspect_process.returncode != 0:
            cmd = [
                "bash",
                "trainer/training_files/scripts/" +
                "create_p3trainfarm_docker_container.bash"]
            self.docker_construct_process = subprocess.Popen(cmd)
            self.docker_construct_process.communicate()
        else:
            print(self._TRAIN_DOCKER_CONTAINER + " - Docker Container FOUND.")
        # Enter docker container
        # Install dependencies
        cmd = [
            "bash",
            "trainer/training_files/scripts/" +
            "prepare_p3trainfarm_docker_container.bash"]
        self.install_depend_process = subprocess.Popen(cmd)
        self.install_depend_process.communicate()

        # If docker_pull_process succeeded,
        # Update p3_trainfarm_verification.json
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)

        if self.install_depend_process.returncode == 0:
            p3_train_verification["isTrainDependenciesInstalled"] = True
        else:
            p3_train_verification["isTrainDependenciesInstalled"] = False
        json_object = json.dumps(p3_train_verification, indent=4)
        with open(_path_to_p3_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def installExporterDependencies(self):
        # Check if docker container exists.
        cmd = [
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
        if self.docker_inspect_process.returncode != 0:
            cmd = [
                "bash",
                "trainer/exporter_files/scripts/" +
                "create_p3exporter_docker_container.bash"]
            self.docker_construct_process = subprocess.Popen(cmd)
            self.docker_construct_process.communicate()
        else:
            print(self._EXPORT_DOCKER_CONTAINER + " - Docker Container FOUND.")
        # Enter docker container
        # Install dependencies
        cmd = [
            "bash",
            "trainer/exporter_files/scripts/" +
            "prepare_p3exporter_docker_container.bash"]
        self.install_depend_process = subprocess.Popen(cmd)
        self.install_depend_process.communicate()

        # If docker_pull_process succeeded,
        # Update p3_trainfarm_verification.json
        _path_to_p3_train_verification = "../config/p3_train_verification.json"
        # Load p3_trainfarm_verification.json
        file = open(_path_to_p3_train_verification)
        p3_train_verification = json.load(file)

        if self.install_depend_process.returncode == 0:
            p3_train_verification["isExportDependenciesInstalled"] = True
        else:
            p3_train_verification["isExportDependenciesInstalled"] = False
        json_object = json.dumps(p3_train_verification, indent=4)
        with open(_path_to_p3_train_verification, 'w') as outfile:
            outfile.write(json_object)
        file.close()

    def copyTrainingFiles(self):
        # Check if docker container exists.
        cmd = [
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
            print(self._TRAIN_DOCKER_CONTAINER + " - Docker Container FOUND.")
        cmd = ["bash", "trainer/training_files/scripts/copy_p3_files.bash"]
        self.copy_process = subprocess.Popen(cmd)
        self.copy_process.communicate()

    def runTraining(self):
        # Check if docker container exists.
        cmd = [
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
            print(self._TRAIN_DOCKER_CONTAINER + " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/training_files/scripts/deploy_p3trainfarm_training.bash"]
        self.training_process = subprocess.Popen(cmd)
        self.training_process.communicate()

    def copyExportFiles(self):
        # Check if docker container exists.
        cmd = [
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
            print(self._EXPORT_DOCKER_CONTAINER + " - Docker Container FOUND.")
        cmd = ["bash", "trainer/exporter_files/scripts/copy_p3_files.bash"]
        self.copy_process = subprocess.Popen(cmd)
        self.copy_process.communicate()

    def runExporter(self):
        # Check if docker container exists.
        cmd = [
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
            print(self._EXPORT_DOCKER_CONTAINER + " - Docker Container FOUND.")
        cmd = [
            "bash",
            "trainer/exporter_files/scripts/deploy_p3exporter_exporter.bash"]
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
            print("ONNX model generated from .pth file. " +
                  "File saved to [data/model/" +
                  TIMESTAMPED_ONNX_FILE_NAME + "]...")
            cmd = ["cp",
                   _FILE_PATH_TO_ONNX_MODEL,
                   "../data/model/" + TIMESTAMPED_ONNX_FILE_NAME]
            self.shift_process = subprocess.Popen(cmd)
            self.shift_process.communicate()
        else:
            print("[ output.onnx ] - MISSING. " +
                  "Something must have failed before this.")
