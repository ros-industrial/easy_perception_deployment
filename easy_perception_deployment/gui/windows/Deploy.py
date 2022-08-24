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

import os
import sys
import json
import subprocess
import logging

from PySide2.QtCore import QSize
from PySide2.QtGui import QIcon
from PySide2.QtWidgets import QComboBox, QFileDialog, QLabel, QTextEdit
from PySide2.QtWidgets import QMessageBox, QPushButton, QWidget

from windows.Counting import CountingWindow
from windows.Tracking import TrackingWindow


class DeployWindow(QWidget):
    '''
    The DeployWindow class is a PySide2 Graphical User Interface (GUI) window
    that is called by MainWindow class in order to configure a custom session
    and write to session_config.json.
    '''
    def __init__(self, debug=False):
        '''
        The constructor.
        Sets the size of the window and configurations for session_config.
        Checks if the session_config.json file exists. If true, configure
        accordingly. Otherwise, assign default values.
        Calls setButtons function to populate window with button.
        '''
        super().__init__()

        self.deploy_logger = logging.getLogger('deploy')

        self.debug = debug

        self._path_to_model = ''
        self._path_to_label_list = ''
        self._input_image_topic = ''

        self._is_running = False

        self._DEPLOY_WIN_H = 400
        self._DEPLOY_WIN_W = 500

        self.setWindowIcon(QIcon("img/epd_desktop.png"))

        self._deploy_process = None
        self._kill_process = None

        self.visualizeFlag = True

        self.useCPU = True

        self._path_to_session_config = ('../config/session_config.json')
        self._path_to_usecase_config = ('../config/usecase_config.json')
        self._path_to_input_image_json_file = (
            '../config/input_image_topic.json')

        self.usecase_list = [
            'Classification',
            'Counting',
            'Color-Matching',
            'Localization',
            'Tracking']

        session_config = None
        usecase_config = None
        if self.doesFileExist(self._path_to_session_config):
            session_config_json_obj = open(self._path_to_session_config)
            session_config = json.load(session_config_json_obj)
        else:
            self.deploy_logger.warning(
                '[ session_config.json ] is missing.' +
                'Assigning default values')
            self._path_to_model = 'filepath/to/onnx/model'
            self._path_to_label_list = 'filepath/to/classes/list/txt'
            self.visualizeFlag = True
            self.useCPU = True

        if self.doesFileExist(self._path_to_usecase_config):
            usecase_config_json_obj = open(self._path_to_usecase_config)
            usecase_config = json.load(usecase_config_json_obj)
        else:
            self.deploy_logger.warning(
                '[usecase_config.json] is missing.'
                'Assigning default Use Case MODE : ' +
                '[CLASSIFICATION] ')
            self.usecase_mode = 0

        try:
            self._path_to_model = session_config["path_to_model"]
            self._path_to_label_list = session_config["path_to_label_list"]
            if session_config["visualizeFlag"] == "visualize":
                self.visualizeFlag = True
            else:
                self.visualizeFlag = False
            if session_config["useCPU"] == "CPU":
                self.useCPU = True
            else:
                self.useCPU = False
        except (KeyError, TypeError) as e:
            self.deploy_logger.exception("[ session_config.json ] - " +
                                         "KeyError or TypeError detected" +
                                         "Assigning default values")
            self._path_to_model = 'filepath/to/onnx/model'
            self._path_to_label_list = 'filepath/to/classes/list/txt'
            self.visualizeFlag = True
            self.useCPU = True

        try:
            self.usecase_mode = int(usecase_config["usecase_mode"])

            if self.usecase_mode < 0 or self.usecase_mode > 4:
                self.deploy_logger.warning(
                    '[ usecase_config.json ] - Invalid Usecase Mode' +
                    ' - FOUND.\n'
                    'Assigning default Use Case MODE : [CLASSIFICATION] ')
                self.usecase_mode = 0

            # Rearranging usecase_list based on saved configuration.
            curr_usecase_mode = self.usecase_list[int(self.usecase_mode)]
            self.usecase_list.remove(curr_usecase_mode)
            self.usecase_list.insert(0, curr_usecase_mode)
        except TypeError:
            self.deploy_logger.exception(
                "[ usecase_config.json ] - " +
                "TypeError detected" +
                "Assigning default Use Case MODE : " +
                "[CLASSIFICATION] ")
            self.usecase_mode = 0

        if self.doesFileExist(self._path_to_input_image_json_file):
            # Load input_image_topic.json
            f = open(self._path_to_input_image_json_file)
            data = json.load(f)
            self._input_image_topic = data['input_image_topic']
        else:
            self._input_image_topic = '/camera/color/image_raw'

        self.setWindowTitle('Deploy')
        self.setGeometry(self._DEPLOY_WIN_W,
                         0,
                         self._DEPLOY_WIN_W,
                         self._DEPLOY_WIN_H)
        self.setFixedSize(self._DEPLOY_WIN_W, self._DEPLOY_WIN_H)

        self.setButtons()
        self.printDeployConfig()

    def printDeployConfig(self):
        '''
        A Non-Return Getter function that prints EPD Deployment
        configurations that are not displayed clearly in EPD GUI, on terminal.
        '''
        self.deploy_logger.info('[- EPD Deployment Configurations -]')
        self.deploy_logger.info('[ ONNX Model ] : ' + self._path_to_model)
        self.deploy_logger.info('[ Label List ] : ' + self._path_to_label_list)
        self.deploy_logger.info(
            '[ Input Image Topic ] : ' + self._input_image_topic)

    def setButtons(self):
        '''A Mutator function that defines all buttons in DeployWindow.'''
        # ONNX Model to set the path to ONNX model and
        # store in session_config.json
        self.model_button = QPushButton('ONNX Model', self)
        self.model_button.setIcon(QIcon('img/model.png'))
        self.model_button.setIconSize(QSize(75, 75))
        self.model_button.setGeometry(
            0,
            0,
            self._DEPLOY_WIN_W/2,
            self._DEPLOY_WIN_H/4)

        index = self._path_to_model.find('data/model')
        if self.doesFileExist('../' + self._path_to_model[index:]):
            self.model_button.setStyleSheet(
                'background-color: rgba(0,150,10,255);')
        else:
            self.model_button.setStyleSheet(
                'background-color: rgba(200,10,0,255);')

        # Label List to set the path to ONNX model
        # and store in session_config.json
        self.list_button = QPushButton('Label List', self)
        self.list_button.setIcon(QIcon('img/label_list.png'))
        self.list_button.setIconSize(QSize(75, 75))
        self.list_button.setGeometry(self._DEPLOY_WIN_W/2,
                                     0,
                                     self._DEPLOY_WIN_W/2,
                                     self._DEPLOY_WIN_H/4)

        index = self._path_to_label_list.find('data/label_list')
        if self.doesFileExist('../' + self._path_to_label_list[index:]):
            self.list_button.setStyleSheet(
                'background-color: rgba(0,150,10,255);')
        else:
            self.list_button.setStyleSheet(
                'background-color: rgba(200,10,0,255);')

        # UseCase Config Dropdown to select usecase mode
        self.usecase_config_button = QComboBox(self)
        self.usecase_config_button.setGeometry(self._DEPLOY_WIN_W/2,
                                               self._DEPLOY_WIN_H/4,
                                               self._DEPLOY_WIN_W/2,
                                               self._DEPLOY_WIN_H/4)
        for usecase in self.usecase_list:
            self.usecase_config_button.addItem(usecase)

        if self.doesFileExist(self._path_to_usecase_config):
            self.usecase_config_button.setStyleSheet(
                'background-color: rgba(0,150,10,255);')
        else:
            self.usecase_config_button.setStyleSheet(
                'background-color: rgba(200,10,0,255);')

        self.visualize_button = QPushButton(self)
        self.visualize_button.setGeometry(0,
                                          self._DEPLOY_WIN_H/4,
                                          self._DEPLOY_WIN_W/2,
                                          self._DEPLOY_WIN_H/4)
        if self.visualizeFlag:
            self.visualize_button.setText('Visualize')
        else:
            self.visualize_button.setText('Action')

        self.register_topic_button = QPushButton(self)
        self.register_topic_button.setGeometry(
            0,
            self._DEPLOY_WIN_H * 2/4,
            self._DEPLOY_WIN_W * 3/8,
            self._DEPLOY_WIN_H/8)
        self.register_topic_button.setText('Register Topic')

        self.topic_button = QTextEdit(self)
        self.topic_button.setGeometry(
            self._DEPLOY_WIN_W * 3/8,
            self._DEPLOY_WIN_H * 2/4,
            self._DEPLOY_WIN_W * 5/8,
            self._DEPLOY_WIN_H/8)
        # Replace use of button with QTextEdit.
        # Read the run.launch.py file.
        # Read line 25 in the file and get the input image topic.
        # Print out the input image topic below.

        self.topic_button.setText(self._input_image_topic)

        self.docker_button = QPushButton(self)
        self.docker_button.setGeometry(0,
                                       self._DEPLOY_WIN_H * 5/8,
                                       self._DEPLOY_WIN_W,
                                       self._DEPLOY_WIN_H/8)
        if self.useCPU is True:
            self.docker_button.setText('CPU')
        else:
            self.docker_button.setText('GPU')

        # Run button to deploy ROS2 package with info
        # from usecase_config.json and session_config.json
        self.run_button = QPushButton('Run', self)
        self.run_button.setIcon(QIcon('img/go.png'))
        self.run_button.setIconSize(QSize(100, 100))
        self.run_button.setGeometry(0,
                                    self._DEPLOY_WIN_H * 3/4,
                                    self._DEPLOY_WIN_W,
                                    self._DEPLOY_WIN_H/4)

        self.visualize_button.clicked.connect(self.setVisualizeFlag)
        self.docker_button.clicked.connect(self.setDockerFlag)
        self.model_button.clicked.connect(self.setModel)
        self.list_button.clicked.connect(self.setLabelList)
        self.usecase_config_button.activated.connect(self.setUseCase)
        self.run_button.clicked.connect(self.deployPackage)
        self.register_topic_button.clicked.connect(self.setImageInput)

    def deployPackage(self):
        '''
        A Mutator function that runs a bash script that
        checks if the _is_running boolean flag is True or not.\n
        If False, run bash script to run ROS2 package with
        session_config.json and usecase_config.json
        Otherwise, run bash script to kill ROS2 package
        processes remotely.
        '''
        if not self._is_running:
            self._deploy_process = subprocess.Popen(['./scripts/deploy.sh',
                                                     str(self.useCPU),
                                                     str(self.visualizeFlag)])
            self.run_button.setText('Stop')
            self.run_button.setIcon(QIcon('img/quit.png'))
            self.run_button.setIconSize(QSize(100, 100))
            self.run_button.updateGeometry()
            self._is_running = True
        else:
            self.deploy_logger.info("Killing epd_test_container docker.")
            self._kill_process = subprocess.Popen(['./scripts/kill.sh'])
            self.run_button.setText('Run')
            self.run_button.setIcon(QIcon('img/go.png'))
            self.run_button.setIconSize(QSize(100, 100))
            self.run_button.updateGeometry()
            self._is_running = False

    def setImageInput(self):
        '''
        A Mutator function that writes to line 25 of
        run.launch.py file based on new image topic.
        '''
        new_image_topic = self.topic_button.toPlainText()
        self.deploy_logger.info(
            'Rewriting Input Image Topic to: ' +
            new_image_topic)

        dict = {"input_image_topic": new_image_topic}
        json_object = json.dumps(dict, indent=4)

        with open(self._path_to_input_image_json_file, 'w') as outfile:
            outfile.write(json_object)

    def doesFileExist(self, input_filepath):
        ''' A Getter function that checks if a given file exists.'''
        if os.path.exists(input_filepath):
            return True
        else:
            self.deploy_logger.warning(
                '[ ' + input_filepath + ' ] does not exist.')
            return False

    def setVisualizeFlag(self):
        '''A function is triggered by the button labelled, Visualize/Action.'''
        self.visualizeFlag = not self.visualizeFlag

        if self.visualizeFlag:
            self.visualize_button.setText('Visualize')
        else:
            self.visualize_button.setText('Action')

        self.visualize_button.updateGeometry()
        self.updateSessionConfig()

    def setDockerFlag(self):
        '''A function is triggered by the button labelled, CPU/GPU.'''
        self.useCPU = not self.useCPU

        if self.useCPU:
            self.docker_button.setText('CPU')
        else:
            self.docker_button.setText('GPU')
        self.docker_button.updateGeometry()
        self.updateSessionConfig()

    def setUseCase(self, index):
        '''A function is triggered by the DropDown Menu labelled, UseCase.'''
        selected_usecase = self.usecase_list[index]

        if selected_usecase == 'Classification':
            if not self.debug:
                msgBox = QMessageBox()
                msgBox.setText('[Classification] Selected.'
                               'No other configuration required.')
                msgBox.exec()

            self.deploy_logger.info('Wrote to ../data/usecase_config.json')
            dict = {"usecase_mode": 0}
            json_object = json.dumps(dict, indent=4)
            with open(self._path_to_usecase_config, 'w') as outfile:
                outfile.write(json_object)

        elif selected_usecase == 'Counting':
            self.counting_window = CountingWindow(self._path_to_label_list,
                                                  self._path_to_usecase_config)
            self.counting_window.show()
        elif selected_usecase == 'Localization':
            dict = {"usecase_mode": 3}
            json_object = json.dumps(dict, indent=4)
            with open(self._path_to_usecase_config, 'w') as outfile:
                outfile.write(json_object)
        elif selected_usecase == 'Tracking':
            self.tracking_window = TrackingWindow(self._path_to_usecase_config)
            self.tracking_window.show()
        elif selected_usecase == 'Color-Matching':
            if not self.debug:
                input_refimage_filepath, ok = (
                    QFileDialog.getOpenFileName(
                        self,
                        'Set the .png/.jpg color image to use',
                        os.path.abspath('../data'),
                        'Image Files (*.png *.jpg *.jpeg)'))
            else:
                input_refimage_filepath = 'dummy_filepath_to_refimage'
                ok = True

            if ok:
                filepath_index = input_refimage_filepath.find('/data')
                path_to_color_template = (
                    '.' +
                    input_refimage_filepath[filepath_index:])
            else:
                self.deploy_logger.warning('No reference color template set.')
                return

            self.deploy_logger.info('Wrote to ../data/usecase_config.json')
            dict = {
                "usecase_mode": 2,
                "path_to_color_template": path_to_color_template
                }
            json_object = json.dumps(dict, indent=4)

            with open(self._path_to_usecase_config, 'w') as outfile:
                outfile.write(json_object)
        else:
            self.deploy_logger.warning('Invalid Use Case')
            sys.exit()

        self.usecase_config_button.setStyleSheet(
            'background-color: rgba(0,150,10,255);')

    def updateSessionConfig(self):
        '''A Mutator function that updates the session_config.json file.'''

        if self.visualizeFlag:
            visualizeFlag_string = "visualize"
        else:
            visualizeFlag_string = "robot"

        if self.useCPU:
            useCPU_string = "CPU"
        else:
            useCPU_string = "GPU"

        dict = {
            "path_to_model": self._path_to_model,
            "path_to_label_list": self._path_to_label_list,
            "visualizeFlag": visualizeFlag_string,
            "useCPU": useCPU_string
            }
        json_object = json.dumps(dict, indent=4)

        with open(self._path_to_session_config, 'w') as outfile:
            outfile.write(json_object)

    def setModel(self):
        '''A function is triggered by the button labelled, ONNX Model.'''
        if not self.debug:
            input_model_filepath, ok = (
                QFileDialog.getOpenFileName(self,
                                            'Set the .onnx model to use',
                                            os.path.abspath('../data'),
                                            'ONNX Model Files (*.onnx)'))
        else:
            input_model_filepath = 'dummy_model_filepath'
            ok = True

        if ok:
            self._path_to_model = input_model_filepath

            index = input_model_filepath.find('/data/model')

            self._path_to_model = '.' + input_model_filepath[index:]
        else:
            self.deploy_logger.warning('No ONNX model set.')
            return

        self.model_button.setStyleSheet(
            'background-color: rgba(0,150,10,255);')
        self.updateSessionConfig()

    def setLabelList(self):
        '''A function is triggered by the button labelled, Label List.'''
        if not self.debug:
            input_classes_filepath, ok = (
                QFileDialog.getOpenFileName(self,
                                            'Set the .json to use',
                                            os.path.abspath('../data'),
                                            'Text Files (*.txt)'))
        else:
            input_classes_filepath = 'dummy_label_list_filepath'
            ok = True

        if ok:
            self._path_to_label_list = input_classes_filepath

            index = input_classes_filepath.find('/data/label_list')

            self._path_to_label_list = '.' + input_classes_filepath[index:]
        else:
            self.deploy_logger.warning('No label list set.')
            return

        self.list_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        self.updateSessionConfig()
