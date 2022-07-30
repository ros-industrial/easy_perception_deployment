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

import os
import subprocess

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
    and write to session_config.txt.
    '''
    def __init__(self, debug=False):
        '''
        The constructor.
        Sets the size of the window and configurations for session_config.
        Checks if the session_config.txt file exists. If true, configure
        accordingly. Otherwise, assign default values.
        Calls setButtons function to populate window with button.
        '''
        super().__init__()

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

        self._path_to_session_config = '../data/session_config.txt'
        self._path_to_usecase_config = '../data/usecase_config.txt'
        self._path_to_launch_file = '../launch/run.launch.py'

        self.usecase_list = ['Classification', 'Counting', 'Color-Matching', 'Localization', 'Tracking']

        # Check if session_config.txt file exists.
        # If does not exist, assign default value.
        if self.doesFileExist(self._path_to_session_config):
            self.session_config = [line.rstrip('\n') for
                                   line in open(self._path_to_session_config)]

            if len(self.session_config) == 4:
                self._path_to_model = self.session_config[0]
                self._path_to_label_list = '.' + self.session_config[1]
                self.visualizeFlag = True if self.session_config[2] == 'visualize' else False
                self.useCPU = True if self.session_config[3] == 'CPU' else False
            else:
                self._path_to_model = 'filepath/to/onnx/model'
                self._path_to_label_list = 'filepath/to/classes/list/txt'
                self.useCPU = True
                self.session_config = [self._path_to_label_list, self._path_to_model, 'visualize', 'CPU']

        else:
            print('[ session_config.txt ] is missing or faulty. Assigning default values')
            self._path_to_model = 'filepath/to/onnx/model'
            self._path_to_label_list = 'filepath/to/classes/list/txt'
            self.useCPU = True
            self.session_config = [self._path_to_label_list, self._path_to_model, 'visualize', 'CPU']


        # Check if usecase_config.txt file exists.
        # If does not exist, assign default value.
        if self.doesFileExist(self._path_to_usecase_config):
            self.usecase_config = [line.rstrip('\n') for
                                   line in open(self._path_to_usecase_config)]
            self.usecase_mode = self.usecase_config[0]
            # Rearranging usecase_list based on saved configuration.
            curr_usecase_mode = self.usecase_list[int(self.usecase_mode)]
            self.usecase_list.remove(curr_usecase_mode)
            self.usecase_list.insert(0, curr_usecase_mode)
        else:
            print('[usecase_config.txt] is missing.'
                  'Assigning default Use Case MODE : [CLASSIFICATION] ')
            self.usecase_mode = 0

        if self.doesFileExist(self._path_to_launch_file):
            self.launch_file = [line.rstrip('\n') for
                                   line in open(self._path_to_launch_file)]
            self._input_image_topic = self.launch_file[24]
            self._input_image_topic = self._input_image_topic[69:-3]
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
        print ('[- EPD Deployment Configurations -]')
        print('[ ONNX Model ] : ' + self._path_to_model )
        print('[ Label List ] : ' + self._path_to_label_list )
        print('[ Input Image Topic ] : ' + self._input_image_topic)

    def setButtons(self):
        '''A Mutator function that defines all buttons in DeployWindow.'''
        # ONNX Model to set the path to ONNX model and store in session_config.txt
        self.model_button = QPushButton('ONNX Model', self)
        self.model_button.setIcon(QIcon('img/model.png'))
        self.model_button.setIconSize(QSize(75, 75))
        self.model_button.setGeometry(0, 0, self._DEPLOY_WIN_W/2, self._DEPLOY_WIN_H/4)

        index = self._path_to_model.find('data/model')
        if self.doesFileExist('../' + self._path_to_model[index:]):
            self.model_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        else:
            self.model_button.setStyleSheet('background-color: rgba(200,10,0,255);')

        # Label List to set the path to ONNX model and store in session_config.txt
        self.list_button = QPushButton('Label List', self)
        self.list_button.setIcon(QIcon('img/label_list.png'))
        self.list_button.setIconSize(QSize(75, 75))
        self.list_button.setGeometry(self._DEPLOY_WIN_W/2,
                                     0,
                                     self._DEPLOY_WIN_W/2,
                                     self._DEPLOY_WIN_H/4)

        index = self._path_to_label_list.find('data/label_list')
        if self.doesFileExist('../' + self._path_to_label_list[index:]):
            self.list_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        else:
            self.list_button.setStyleSheet('background-color: rgba(200,10,0,255);')

        # UseCase Config Dropdown to select usecase mode
        self.usecase_config_button = QComboBox(self)
        self.usecase_config_button.setGeometry(self._DEPLOY_WIN_W/2,
                                               self._DEPLOY_WIN_H/4,
                                               self._DEPLOY_WIN_W/2,
                                               self._DEPLOY_WIN_H/4)
        for usecase in self.usecase_list:
            self.usecase_config_button.addItem(usecase)

        if self.doesFileExist(self._path_to_usecase_config):
            self.usecase_config_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        else:
            self.usecase_config_button.setStyleSheet('background-color: rgba(200,10,0,255);')

        # UseCase button to select use case and write to usecase_config.txt
        self.usecase_config_label = QLabel(self)
        self.usecase_config_label.setText('Use Case =')
        self.usecase_config_label.move(self._DEPLOY_WIN_W/2 - 90,
                                       self._DEPLOY_WIN_H/4 + 40)
        self.usecase_config_label.setBuddy(self.usecase_config_button)

        self.visualize_button = QPushButton(self)
        self.visualize_button.setGeometry(0,
                                          self._DEPLOY_WIN_H/4,
                                          self._DEPLOY_WIN_W/4,
                                          self._DEPLOY_WIN_H/4)
        if self.visualizeFlag:
            self.visualize_button.setText('Visualize')
        else:
            self.visualize_button.setText('Action')

        self.register_topic_button = QPushButton(self)
        self.register_topic_button.setGeometry(0,
                                       self._DEPLOY_WIN_H * 2/4,
                                       self._DEPLOY_WIN_W * 3/8,
                                       self._DEPLOY_WIN_H/8)
        self.register_topic_button.setText('Register Topic')

        self.topic_button = QTextEdit(self)
        self.topic_button.setGeometry(self._DEPLOY_WIN_W * 3/8,
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
        # from usecase_config.txt and session_config.txt
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
        A Mutator function that runs a bash script that checks if the _is_running
        boolean flag is True or not.\n
        If False, run bash script to run ROS2 package with session_config.txt and
        usecase_config.txt
        Otherwise, run bash script to kill ROS2 package processes remotely.
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
            print ("Killing epd_test_container docker.")
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
        print('Rewriting Input Image Topic to: ' + new_image_topic)

        self.launch_file[24] = self.launch_file[24][:52] + new_image_topic + self.launch_file[24][-3:]

        with open(self._path_to_launch_file, 'w') as filehandle:
            for line in self.launch_file:
                filehandle.write(line + '\n')

    def doesFileExist(self, input_filepath):
        ''' A Getter function that checks if a given file exists.'''
        if os.path.exists(input_filepath):
            return True
        else:
            print('WARNING: [ ', input_filepath, ' ] ', ' does not exist.')
            return False

    def setVisualizeFlag(self):
        '''A function is triggered by the button labelled, Visualize/Action.'''
        self.visualizeFlag = not self.visualizeFlag

        if self.visualizeFlag:
            self.visualize_button.setText('Visualize')
            self.session_config[2] = 'visualize'
        else:
            self.visualize_button.setText('Action')
            self.session_config[2] = 'robot'
        self.visualize_button.updateGeometry()
        self.updateSessionConfig()

    def setDockerFlag(self):
        '''A function is triggered by the button labelled, CPU/GPU.'''
        self.useCPU = not self.useCPU

        if self.useCPU:
            self.docker_button.setText('CPU')
            self.session_config[3] = 'CPU'
        else:
            self.docker_button.setText('GPU')
            self.session_config[3] = 'GPU'
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
            print('Wrote to ../data/usecase_config.txt')
            with open(self._path_to_usecase_config, 'w') as filehandle:
                filehandle.write('0\n')
        elif selected_usecase == 'Counting':
            self.counting_window = CountingWindow(self._path_to_label_list,
                                                  self._path_to_usecase_config)
            self.counting_window.show()
        elif selected_usecase == 'Localization':
            with open(self._path_to_usecase_config, 'w') as filehandle:
                filehandle.write('3\n')
        elif selected_usecase == 'Tracking':
            self.tracking_window = TrackingWindow(self._path_to_usecase_config)
            self.tracking_window.show()
        else:
            if not self.debug:
                input_refimage_filepath, ok = (
                    QFileDialog.getOpenFileName(self,
                                                'Set the .png/.jpg color image to use',
                                                os.path.abspath('../data'),
                                                'Image Files (*.png *.jpg *.jpeg)'))
            else:
                input_refimage_filepath = 'dummy_filepath_to_refimage'
                ok = True

            self.usecase_config.clear()
            self.usecase_config.append(str(2))
            if ok:

                filepath_index = input_refimage_filepath.find('/data')

                self.usecase_config.append('.' + input_refimage_filepath[filepath_index:])
            else:
                print('No reference color template set.')
                return
            print('Wrote to ../data/usecase_config.txt')
            with open(self._path_to_usecase_config, 'w') as filehandle:
                for ele in self.usecase_config:
                    filehandle.write('%s\n' % ele)

        self.usecase_config_button.setStyleSheet('background-color: rgba(0,150,10,255);')

    def updateSessionConfig(self):
        '''A Mutator function that updates the session_config.txt file.'''
        with open(self._path_to_session_config, 'w') as filehandle:
            for ele in self.session_config:
                filehandle.write('%s\n' % ele)

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

            self.session_config[0] = '.' + input_model_filepath[index:]
        else:
            print('No ONNX model set.')
            return

        self.model_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        self.updateSessionConfig()

    def setLabelList(self):
        '''A function is triggered by the button labelled, Label List.'''
        if not self.debug:
            input_classes_filepath, ok = (
                QFileDialog.getOpenFileName(self,
                                            'Set the .txt to use',
                                            os.path.abspath('../data'),
                                            'Text Files (*.txt)'))
        else:
            input_classes_filepath = 'dummy_label_list_filepath'
            ok = True

        if ok:
            self._path_to_label_list = input_classes_filepath

            index = input_classes_filepath.find('/data/label_list')

            self.session_config[1] = '.' + input_classes_filepath[index:]
        else:
            print('No label list set.')
            return

        self.list_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        self.updateSessionConfig()
