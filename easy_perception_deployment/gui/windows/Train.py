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
import subprocess
from ast import literal_eval as make_tuple

from PySide2.QtCore import QSize
from PySide2.QtGui import QIcon
from PySide2.QtWidgets import (QComboBox, QFileDialog, QInputDialog, QLabel,
                               QLineEdit, QPushButton, QWidget)
from trainer.P2Trainer import P2Trainer
from trainer.P3Trainer import P3Trainer


class TrainWindow(QWidget):
    '''
    The TrainWindow class is a PySide2 Graphical User Interface (GUI) window
    that is called by MainWindow class in order to configure a custom training
    session and initiates training for a selected Precision-Level.
    '''
    def __init__(self, debug=False):
        '''
        The constructor.
        Sets the size of the window and configurations for a training session.
        Calls setButtons function to populate window with button.
        '''
        super().__init__()

        self.debug = debug

        self._TRAIN_WIN_H = 650
        self._TRAIN_WIN_W = 500
        self._ROW_THICKNESS = 100

        self.setWindowIcon(QIcon("img/epd_desktop.png"))

        self.model_name = ''
        self._model_list = []
        self._label_list = []
        self._precision_level = 2

        self._path_to_dataset = ''
        self._path_to_label_list = ''
        self._is_valid_dataset = False
        self.buttonConnected = False

        self.label_process = None

        self._is_model_ready = False
        self._is_dataset_linked = False
        self._is_dataset_labelled = False
        self._is_labellist_linked = False

        self.label_train_process = None
        self.label_val_process = None

        self.max_iteration = 3000
        self.checkpoint_period = 200
        self.test_period = 200
        self.steps = '(1000, 1500, 2000, 2500)'

        self.setWindowTitle('Train')
        self.setGeometry(
            self._TRAIN_WIN_W*2,
            0,
            self._TRAIN_WIN_W,
            self._TRAIN_WIN_H)
        self.setFixedSize(self._TRAIN_WIN_W, self._TRAIN_WIN_H)

        self.setButtons()

    def setButtons(self):
        '''A Mutator function that defines all buttons in TrainWindow.'''

        self.p2_button = QPushButton('P2', self)
        self.p2_button.setGeometry(0, 0, 50, 100)
        self.p2_button.setStyleSheet(
            'background-color: rgba(180,180,180,255);')

        self.p3_button = QPushButton('P3', self)
        self.p3_button.setGeometry(50, 0, 50, 100)

        # Model dropdown menu to select Precision Level specific model
        self.model_selector = QComboBox(self)
        self.model_selector.setGeometry(
            self._TRAIN_WIN_W-150,
            0,
            150,
            self._ROW_THICKNESS)
        self.model_selector.setStyleSheet('background-color: red;')
        self.populateModelSelector()

        # Labeller button to initiate labelme
        self.label_button = QPushButton('Label Dataset', self)
        self.label_button.setIcon(QIcon('img/label_labelme.png'))
        self.label_button.setIconSize(QSize(50, 50))
        self.label_button.setGeometry(
            0,
            200,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS)
        self.label_button.setStyleSheet(
            'background-color: rgba(0,200,10,255);')
        if self._precision_level == 1:
            self.label_button.hide()

        self.generate_button = QPushButton('Generate Dataset', self)
        self.generate_button.setIcon(QIcon('img/label_generate.png'))
        self.generate_button.setIconSize(QSize(50, 50))
        self.generate_button.setGeometry(
            self._TRAIN_WIN_W/2,
            200,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS)
        self.generate_button.setStyleSheet(
            'background-color: rgba(0,200,10,255);')

        # Labeller button to initiate labelme
        self.validate_button = QPushButton('Validate Dataset', self)
        self.validate_button.setIcon(QIcon('img/validate.png'))
        self.validate_button.setIconSize(QSize(50, 50))
        self.validate_button.setGeometry(
            self._TRAIN_WIN_W/2,
            300,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS)

        # Dataset button to prompt input via FileDialogue
        self.dataset_button = QPushButton('Choose Dataset', self)
        self.dataset_button.setIcon(QIcon('img/dataset.png'))
        self.dataset_button.setIconSize(QSize(50, 50))
        self.dataset_button.setGeometry(
            0,
            300,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS)
        self.dataset_button.setStyleSheet('background-color: red;')

        # Start Training button to start and display training process
        self.train_button = QPushButton('Train', self)
        self.train_button.setIcon(QIcon('img/train.png'))
        self.train_button.setIconSize(QSize(75, 75))
        self.train_button.setGeometry(
            0,
            self._TRAIN_WIN_H-100,
            self._TRAIN_WIN_W,
            self._ROW_THICKNESS)
        self.train_button.setStyleSheet(
            'background-color: rgba(180,180,180,255);')

        # Set Label List
        self.list_button = QPushButton('Choose Label List', self)
        self.list_button.setIcon(QIcon('img/label_list.png'))
        self.list_button.setIconSize(QSize(75, 75))
        self.list_button.setGeometry(
            0,
            100,
            self._TRAIN_WIN_W,
            self._ROW_THICKNESS)
        self.list_button.setStyleSheet(
            'background-color: rgba(200,10,0,255);')

        self.training_config_label = QLabel(self)
        self.training_config_label.setText('Training Parameters')
        self.training_config_label.move(self._TRAIN_WIN_W/2 - 65, 415)

        self.maxiter_button = QPushButton('MAX ITERATION', self)
        self.maxiter_button.setGeometry(
            0,
            450,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS/2)
        self.checkpointp_button = QPushButton('CHECKPOINT PERIOD', self)
        self.checkpointp_button.setGeometry(
            self._TRAIN_WIN_W/2,
            450,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS/2)
        self.steps_button = QPushButton('STEPS', self)
        self.steps_button.setGeometry(
            0,
            500,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS/2)
        self.testp_button = QPushButton('TEST PERIOD', self)
        self.testp_button.setGeometry(
            self._TRAIN_WIN_W/2,
            500,
            self._TRAIN_WIN_W/2,
            self._ROW_THICKNESS/2)

        self.p2_button.clicked.connect(self.setP2)
        self.p3_button.clicked.connect(self.setP3)

        self.model_selector.activated.connect(self.setModel)
        self.dataset_button.clicked.connect(self.setDataset)
        self.label_button.clicked.connect(self.runLabelme)
        self.generate_button.clicked.connect(self.conformDatasetToCOCO)
        self.validate_button.clicked.connect(self.validateDataset)
        self.list_button.clicked.connect(self.setLabelList)

        self.maxiter_button.clicked.connect(self.setMaxIteration)
        self.checkpointp_button.clicked.connect(self.setCheckPointPeriod)
        self.testp_button.clicked.connect(self.setTestPeriod)
        self.steps_button.clicked.connect(self.setSteps)

    def setP2(self):
        '''A function that is triggered by the button labelled, P2.'''
        self._precision_level = 2
        self.populateModelSelector()
        self.initModel()
        self.label_button.show()
        self.generate_button.show()
        self.p2_button.setStyleSheet(
            'background-color: rgba(180,180,180,255);')
        self.p3_button.setStyleSheet('background-color: white;')
        self.disconnectTrainingButton()

    def setP3(self):
        '''A function that is triggered by the button labelled, P3.'''
        self._precision_level = 3
        self.populateModelSelector()
        self.initModel()
        self.label_button.show()
        self.generate_button.show()
        self.p3_button.setStyleSheet(
            'background-color: rgba(180,180,180,255);')
        self.p2_button.setStyleSheet('background-color: white;')
        self.disconnectTrainingButton()

    def setModel(self, index):
        '''A function that is triggered by
        the DropDown Menu option, Model.
        '''
        self.model_name = self.model_selector.itemText(index)

        self.model_selector.setStyleSheet(
            'background-color: rgba(0,200,10,255);')
        self._is_model_ready = True
        self.validateTraining()
        print('Set Model to ', self.model_name)

    def setLabelList(self):
        '''A function that is triggered by Choose Label List button.'''
        if not self.debug:
            input_classes_filepath, ok = (
                QFileDialog.getOpenFileName(
                    self,
                    'Set the .txt to use',
                    os.path.abspath('../data'),
                    'Text Files (*.txt)'))
        else:
            input_classes_filepath = '../data/label_list/coco_classes.txt'
            ok = True

        if ok:
            self._path_to_label_list = input_classes_filepath
            self._label_list = [
                line.rstrip('\n') for line in
                open(input_classes_filepath)]
        else:
            print('[ WARNING ] - No label list set.')
            return
        self.list_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        self._is_labellist_linked = True
        self.validateTraining()

    def setDataset(self):
        '''A function that is triggered by Choose Dataset button.'''
        if not self.debug:
            new_filepath_to_dataset = (
                QFileDialog.getExistingDirectory(
                    self,
                    'Set directory of the dataset',
                    os.path.abspath('../data'),
                    QFileDialog.ShowDirsOnly
                    | QFileDialog.
                    DontResolveSymlinks))

        else:
            new_filepath_to_dataset = '../data/datasets'

        if os.path.isdir(new_filepath_to_dataset):
            self._path_to_dataset = new_filepath_to_dataset
            # Set button color to green
            self._is_dataset_linked = True
            self.dataset_button.setStyleSheet(
                'background-color: rgba(0,200,10,255);')
        else:
            # Set button color to red
            print('[ WARNING ] - Dataset path does not exist.')
            self.dataset_button.setStyleSheet('background-color: red;')

        self.validateTraining()

    def setMaxIteration(self):
        if not self.debug:
            max_iteration, ok = QInputDialog().getInt(
                self,
                "MAX ITERATION",
                "No. of Training Epochs:",
                self.max_iteration)
        else:
            ok = True
            max_iteration = self.max_iteration
        if ok:
            self.max_iteration = max_iteration
            print("Setting Max Iteration to", self.max_iteration)

    def setCheckPointPeriod(self):
        if not self.debug:
            checkpoint_p, ok = QInputDialog().getInt(
                self,
                "CHECKPOINT PERIOD",
                "Interval to Save Model:",
                self.checkpoint_period)
        else:
            ok = True
            checkpoint_p = self.checkpoint_period
        if ok:
            self.checkpoint_period = checkpoint_p
            print("Setting Checkpoint Period to", self.checkpoint_period)

    def setTestPeriod(self):
        if not self.debug:
            test_p, ok = QInputDialog().getInt(
                self,
                "TEST PERIOD",
                "Interval to Test Model:",
                self.test_period)
        else:
            ok = True
            test_p = self.test_period
        if ok:
            self.test_period = test_p
            print("Setting Test Period to", self.test_period)

    def setSteps(self):
        if not self.debug:
            steps, ok = QInputDialog().getText(
                self,
                "STEPS",
                "Tuple of Epochs to Decrease " +
                "Learning Rate Eg. (1000, 2000):",
                QLineEdit.Normal,
                self.steps)
        else:
            ok = False
            steps = self.steps

        if ok:
            # Check if input is a valid tuple
            try:
                local_steps = make_tuple(steps)
            except (ValueError, SyntaxError):
                print("[ WARNING ] - Invalid tuple given. " +
                      "Reassigning default...")
            self.steps = steps
            print("Setting Steps to", self.steps)

    def runLabelme(self):
        '''A function that is triggered by Label Dataset button.'''
        self.label_process = subprocess.Popen(['labelme'])
        self.validateTraining()

    def initModel(self):
        '''
        A Mutator function that sets the model_name
        to the first model available
        whenever the precision level changes.
        '''
        self.model_name = self._model_list[0]
        self._is_model_ready = True
        print('Setting Model to ', self.model_name)

    def validateTraining(self):
        '''
        A Mutator function that evaulates necessary boolean flags
        that are all required to allow proper training given
        a certain requested precision level.
        '''
        # Perform 4 checks to ensure
        # all data is available for Training to start without issue.

        if not self._is_model_ready:
            print('[ WARNING ] - No model provided. Please choose Model.')
            self.train_button.setStyleSheet(
                'background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return
        if not self._is_dataset_linked:
            print('[ WARNING ] - Dataset directory not provided. ' +
                  'Please choose Dataset.')
            self.train_button.setStyleSheet(
                'background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return
        if not self._is_labellist_linked:
            print('[ WARNING ] - Label List not provided. ' +
                  'Please choose Label List.')
            self.train_button.setStyleSheet(
                'background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        if not self._is_dataset_labelled:
            print('[ WARNING ] - Dataset not properly restructured.' +
                  'Please restructure Dataset.')
            self.train_button.setStyleSheet(
                'background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        if not self._is_dataset_labelled:
            print('[ WARNING ] - Dataset not labelled properly. ' +
                  'Please label Dataset.')
            self.train_button.setStyleSheet(
                'background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        self.train_button.setStyleSheet('background-color: white;')
        self.connectTrainingButton()

    def validateDataset(self):
        '''A function that is triggered by Validate Dataset button.'''
        if self._precision_level == 2:
            isDatasetNamedRight = 'custom_dataset'
            os.path.basename(self._path_to_dataset) == 'custom_dataset'
            trainDirExists = os.path.exists(
                self._path_to_dataset + '/train_dataset')
            valDirExists = os.path.exists(
                self._path_to_dataset + '/val_dataset')
            # Check if the dataset folder has the following structure
            if trainDirExists and valDirExists and isDatasetNamedRight:
                self._is_dataset_labelled = True
                self.validate_button.setStyleSheet(
                    'background-color: rgba(0,200,10,255);')
            else:
                self._is_dataset_labelled = False
                print('[ ERROR ] - Please ensure there is /train_dataset' +
                      'and /val_dataset sub-directories' +
                      'in the selected dataset directory.')
        elif self._precision_level == 3:
            isDatasetNamedRight = 'custom_dataset'
            os.path.basename(self._path_to_dataset) == 'custom_dataset'
            trainDirExists = os.path.exists(
                self._path_to_dataset + '/train_dataset')
            valDirExists = os.path.exists(
                self._path_to_dataset + '/val_dataset')
            # Check if the dataset folder has the following structure
            if trainDirExists and valDirExists and isDatasetNamedRight:
                self._is_dataset_labelled = True
                self.validate_button.setStyleSheet(
                    'background-color: rgba(0,200,10,255);')
            else:
                self._is_dataset_labelled = False
                print('[ ERROR ] - Please ensure there is /train_dataset' +
                      'and /val_dataset sub-directories' +
                      'in the selected dataset directory.')

        self.validateTraining()

    def startTraining(self):
        '''A function that is triggered by the button labelled, Train.'''
        self.disconnectTrainingButton()
        self.train_button.setText('Training In Progress')
        self.train_button.updateGeometry()

        if self._precision_level == 1:
            print("[ Deprecation Notice ] - Precision Level 1 features " +
                  "has been deprecated in EPD v0.3.0.")
            print("Please use Precision Level 1 and 2 features instead.")
        elif self._precision_level == 2:
            p2_trainer = P2Trainer(self._path_to_dataset,
                                   self.model_name,
                                   self._label_list,
                                   self.max_iteration,
                                   self.checkpoint_period,
                                   self.test_period,
                                   self.steps)
            p2_trainer.train(False)
        else:
            p3_trainer = P3Trainer(self._path_to_dataset,
                                   self.model_name,
                                   self._label_list,
                                   self.max_iteration,
                                   self.checkpoint_period,
                                   self.test_period,
                                   self.steps)
            p3_trainer.train(False)
            p3_trainer.export(False)

        self.train_button.setText('Train')
        self.train_button.updateGeometry()

    def conformDatasetToCOCO(self):
        '''A function that is triggered by Generate Dataset button.'''
        if not self.debug:
            path_to_labelled = (
                QFileDialog.getExistingDirectory(
                    self,
                    'Select your labeled dataset.',
                    os.path.abspath('../data'),
                    QFileDialog.ShowDirsOnly
                    | QFileDialog.DontResolveSymlinks))
        else:
            path_to_labelled = '../data/datasets/p2p3_dummy_dataset'
        # Check if every image in given folder
        trainDirExists = os.path.exists(path_to_labelled + '/train_dataset')
        valDirExists = os.path.exists(path_to_labelled + '/val_dataset')

        outputTrainDir = '../data/datasets/custom_dataset/train_dataset'
        outputValDir = '../data/datasets/custom_dataset/val_dataset'

        if trainDirExists and valDirExists:
            self.label_train_process = (
                subprocess.Popen([
                    'python',
                    'dataset/labelme2coco.py',
                    '--labels',
                    self._path_to_label_list,
                    path_to_labelled + '/train_dataset',
                    outputTrainDir]))
            if not self.debug:
                self.label_train_process.communicate()
            self.label_val_process = (
                subprocess.Popen([
                    'python',
                    'dataset/labelme2coco.py',
                    '--labels',
                    self._path_to_label_list,
                    path_to_labelled + '/val_dataset',
                    outputValDir]))
            if not self.debug:
                self.label_val_process.communicate()
        else:
            print('[ WARNING ] - Faulty labelled dataset detected.')

    def populateModelSelector(self):
        '''
        A Mutator function that populates the DropDown Menu labelled, Choose
        Model with all available pretrained models from PyTorch model zoo.
        '''
        # Implement different model list based on different precision level.
        if self._precision_level == 2:
            self._model_list = [
                line.rstrip('\n') for line in
                open('./lists/p2_model_list.txt')]
        elif self._precision_level == 3:
            self._model_list = [
                line.rstrip('\n') for line in
                open('./lists/p3_model_list.txt')]

        self.model_selector.clear()
        for model in self._model_list:
            self.model_selector.addItem(model)

    def connectTrainingButton(self):
        '''
        A Mutator function that allows the Train button,
        to be used by the user.
        '''
        if not self.buttonConnected:
            self.train_button.clicked.connect(self.startTraining)
            self.buttonConnected = True

    def disconnectTrainingButton(self):
        '''
        A Mutator function that disallows the Train button,
        to be used by the user.
        '''
        if self.buttonConnected:
            try:
                self.train_button.clicked.disconnect(self.startTraining)
            except Exception:
                pass
            self.buttonConnected = False
