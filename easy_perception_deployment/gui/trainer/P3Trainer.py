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

from datetime import date

import subprocess


class P3Trainer:
    '''
    The Precision-Level 3 (P3) Trainer class object instantiates a training
    session using PyTorch's MaskRCNN-Benchmark for taking an input dataset and
    maskrcnn model from PyTorch model zoo in order to generate a
    custom-trained P3 ONNX model file.\n
    This model can then be deployed as a ROS2 package.
    '''
    def __init__(self, path_to_dataset, model_name, label_list):
        '''
        The constructor.
        Sets all the required fixed path to various files needed to start
        a training session.\n
        Calls setNumClassesInTrainingConfig function.
        '''
        self.model_name = model_name
        self.label_list = label_list

        self.create_process = None
        self.run_process = None
        self.build_export_process = None
        self.export_process = None

        self.path_to_dataset = path_to_dataset
        self.path_to_modif = 'trainer/training_files/modified_paths_catalog.py'
        self.path_to_training_config = 'trainer/training_files/maskrcnn_training.yaml'
        self.path_to_export_config = 'trainer/exporter_files/maskrcnn_export.yaml'
        self.path_to_trim_tools = 'trainer/training_files/trim_mask_rcnn.py'
        self.path_to_remove_init_tool = 'trainer/exporter_files/remove_initializer.py'
        self.path_to_export_modif = 'trainer/exporter_files/export_to_p3_onnx.py'

        self.setNumClassesInTrainingConfig()

    def setNumClassesInTrainingConfig(self):
        '''
        A Mutator function that modifies the various training session config
        files.
        '''
        a_file = open(self.path_to_training_config, 'r')
        b_file = open(self.path_to_export_config, 'r')
        training_config_lines = a_file.readlines()
        export_config_lines = b_file.readlines()

        isCOCOFormat = False
        for label in self.label_list:
            if label == '__ignore__':
                isCOCOFormat = True
                break
        if isCOCOFormat:
            custom_class_no = len(self.label_list)
        else:
            custom_class_no = len(self.label_list) + 2

        modif_line_index = 22
        for i in range(0, len(training_config_lines)):
            if 'NUM_CLASSES:' in training_config_lines[i]:
                modif_line_index = i
                break

        modif_line = ('    NUM_CLASSES: ' +
                      str(custom_class_no) +
                      ' #Change to your number of objects +2\n')

        training_config_lines[modif_line_index] = modif_line

        a_file = open(self.path_to_training_config, 'w')
        a_file.writelines(training_config_lines)
        a_file.close()

        for i in range(0, len(export_config_lines)):
            if 'NUM_CLASSES:' in export_config_lines[i]:
                modif_line_index = i
                break

        modif_line = ('    NUM_CLASSES: ' +
                      str(custom_class_no) +
                      ' #Change to your number of objects +2\n')

        export_config_lines[modif_line_index] = modif_line

        b_file = open(self.path_to_export_config, 'w')
        b_file.writelines(export_config_lines)
        b_file.close()

    def train(self, debug):
        '''
         A Mutator function that conducts a fixed 300-epoch training session to
          generate the custom-trained P3 ONNX model using bash scripts.\n
          Calls createTrainFarm function.\n
          Calls runTrainFarm function.\n.
          Calls exportONNX function.
        '''
        self.createTrainFarm(debug)
        self.runTrainFarm(debug)
        self.createExportFarm(debug)
        self.runExportFarm(debug)

    def createTrainFarm(self, debug):
        '''
        A Mutator function that runs a bash script that downloads and creates the
        necessary environment for a MaskRCNN-Benchmark training session.
        '''
        self.create_process = subprocess.Popen([
                              './trainer/training_files/scripts/install_p3trainfarm.bash',
                              self.path_to_dataset,
                              self.path_to_modif,
                              self.path_to_training_config,
                              self.path_to_trim_tools])
        if not debug:
            self.create_process.communicate()

    def runTrainFarm(self, debug):
        '''
        A Mutator function that runs a bash script that utilizes the environment
        created in the createTrainFarm to run training session.
        '''
        self.run_process = subprocess.Popen([
                              './trainer/training_files/scripts/run_p3trainfarm.bash',
                              self.model_name,
                              str(date.today()),
                              self.path_to_dataset,
                              self.path_to_training_config])
        if not debug:
            self.run_process.communicate()

    def createExportFarm(self, debug):
        '''
        A Mutator function that runs a bash script that downloads, creates
        an Anaconda3 environment called, p3_exporter for exporting the trained
        .pth file to the final ONNX model file.
        '''
        self.build_export_process = subprocess.Popen([
                              './trainer/exporter_files/scripts/install_p3exporter.bash',
                              self.model_name,
                              str(date.today()),
                              self.path_to_export_config,
                              self.path_to_remove_init_tool,
                              self.path_to_export_modif])
        if not debug:
            self.build_export_process.communicate()

    def runExportFarm(self, debug):
        '''
        A Mutator function that runs a bash script that runs p3_exporter
        environment for exporting the trained .pth file to the final ONNX
        model file.
        '''
        self.export_process = subprocess.Popen([
                              './trainer/exporter_files/scripts/run_p3exporter.bash',
                              self.model_name,
                              str(date.today()),
                              self.path_to_export_config])
        if not debug:
            self.export_process.communicate()
