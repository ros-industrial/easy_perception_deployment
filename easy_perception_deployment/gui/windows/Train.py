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
from PySide2.QtWidgets import QComboBox, QFileDialog, QLabel, QPushButton, QWidget

# from trainer.P1Trainer import P1Trainer
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

        self._TRAIN_WIN_H = 500
        self._TRAIN_WIN_W = 500

        self.model_name = ''
        self._model_list = []
        self._label_list = []
        self._precision_level = 1

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

        self.setWindowTitle('Train')
        self.setGeometry(self._TRAIN_WIN_W*2, 0, self._TRAIN_WIN_W, self._TRAIN_WIN_H)
        self.setFixedSize(self._TRAIN_WIN_W, self._TRAIN_WIN_H)

        self.setButtons()

    def setButtons(self):
        '''A Mutator function that defines all buttons in TrainWindow.'''
        self.p1_button = QPushButton('P1', self)
        self.p1_button.setGeometry(0, 0, 50, 100)
        self.p1_button.setStyleSheet('background-color: rgba(180,180,180,255);')

        self.p2_button = QPushButton('P2', self)
        self.p2_button.setGeometry(50, 0, 50, 100)

        self.p3_button = QPushButton('P3', self)
        self.p3_button.setGeometry(100, 0, 50, 100)

        # Model dropdown menu to select Precision Level specific model
        self.model_selector = QComboBox(self)
        self.model_selector.setGeometry(self._TRAIN_WIN_W-150, 0, 150, 100)
        self.model_selector.setStyleSheet('background-color: red;')
        self.populateModelSelector()

        self.model_selector_label = QLabel(self)
        self.model_selector_label.setText('Choose Model =')
        self.model_selector_label.move(220, 40)

        # Labeller button to initiate labelme
        self.label_button = QPushButton('Label Dataset', self)
        self.label_button.setIcon(QIcon('img/label.png'))
        self.label_button.setIconSize(QSize(50, 50))
        self.label_button.setGeometry(0, 200, self._TRAIN_WIN_W/2, 100)
        self.label_button.setStyleSheet('background-color: rgba(0,200,10,255);')
        if self._precision_level == 1:
            self.label_button.hide()

        self.generate_button = QPushButton('Generate Dataset', self)
        self.generate_button.setIcon(QIcon('img/label.png'))
        self.generate_button.setIconSize(QSize(50, 50))
        self.generate_button.setGeometry(self._TRAIN_WIN_W/2, 200, self._TRAIN_WIN_W/2, 100)
        self.generate_button.setStyleSheet('background-color: rgba(0,200,10,255);')
        if self._precision_level == 1:
            self.generate_button.hide()

        # Labeller button to initiate labelme
        self.validate_button = QPushButton('Validate Dataset', self)
        self.validate_button.setIcon(QIcon('img/validate.png'))
        self.validate_button.setIconSize(QSize(50, 50))
        self.validate_button.setGeometry(self._TRAIN_WIN_W/2, 300, self._TRAIN_WIN_W/2, 100)

        # Dataset button to prompt input via FileDialogue
        self.dataset_button = QPushButton('Choose Dataset', self)
        self.dataset_button.setIcon(QIcon('img/dataset.png'))
        self.dataset_button.setIconSize(QSize(50, 50))
        self.dataset_button.setGeometry(0, 300, self._TRAIN_WIN_W/2, 100)
        self.dataset_button.setStyleSheet('background-color: red;')

        # Start Training button to start and display training process
        self.train_button = QPushButton('Train', self)
        self.train_button.setIcon(QIcon('img/train.png'))
        self.train_button.setIconSize(QSize(75, 75))
        self.train_button.setGeometry(0, self._TRAIN_WIN_H-100, self._TRAIN_WIN_W, 100)
        self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')

        # Set Label List
        self.list_button = QPushButton('Choose Label List', self)
        self.list_button.setIcon(QIcon('img/label_list.png'))
        self.list_button.setIconSize(QSize(75, 75))
        self.list_button.setGeometry(0, 100, self._TRAIN_WIN_W, 100)
        self.list_button.setStyleSheet('background-color: rgba(200,10,0,255);')

        self.p1_button.clicked.connect(self.setP1)
        self.p2_button.clicked.connect(self.setP2)
        self.p3_button.clicked.connect(self.setP3)

        self.model_selector.activated.connect(self.setModel)
        self.dataset_button.clicked.connect(self.setDataset)
        self.label_button.clicked.connect(self.runLabelme)
        self.generate_button.clicked.connect(self.conformDatasetToCOCO)
        self.validate_button.clicked.connect(self.validateDataset)
        self.list_button.clicked.connect(self.setLabelList)

    def setP1(self):
        '''A function that is triggered by the button labelled, P1.'''
        self._precision_level = 1
        self.populateModelSelector()
        self.initModel()
        self.label_button.hide()
        self.generate_button.hide()
        self.p1_button.setStyleSheet('background-color: rgba(180,180,180,255);')
        self.p2_button.setStyleSheet('background-color: white;')
        self.p3_button.setStyleSheet('background-color: white;')
        self.disconnectTrainingButton()
        print('Set Precision Level at: ', self._precision_level)

    def setP2(self):
        '''A function that is triggered by the button labelled, P2.'''
        self._precision_level = 2
        self.populateModelSelector()
        self.initModel()
        self.label_button.show()
        self.generate_button.show()
        self.p2_button.setStyleSheet('background-color: rgba(180,180,180,255);')
        self.p1_button.setStyleSheet('background-color: white;')
        self.p3_button.setStyleSheet('background-color: white;')
        self.disconnectTrainingButton()
        print('Set Precision Level at: ', self._precision_level)

    def setP3(self):
        '''A function that is triggered by the button labelled, P3.'''
        self._precision_level = 3
        self.populateModelSelector()
        self.initModel()
        self.label_button.show()
        self.generate_button.show()
        self.p3_button.setStyleSheet('background-color: rgba(180,180,180,255);')
        self.p1_button.setStyleSheet('background-color: white;')
        self.p2_button.setStyleSheet('background-color: white;')
        self.disconnectTrainingButton()
        print('Set Precision Level at: ', self._precision_level)

    def setModel(self, index):
        '''A function that is triggered by the DropDown Menu labelled, Model.'''
        self.model_name = self.model_selector.itemText(index)

        self.model_selector.setStyleSheet('background-color: rgba(0,200,10,255);')
        self._is_model_ready = True
        self.validateTraining()
        print('Set Model to ', self.model_name)

    def setLabelList(self):
        '''A function that is triggered by the button labelled, Choose Label List.'''
        if not self.debug:
            input_classes_filepath, ok = QFileDialog.getOpenFileName(self,
                                                                     'Set the .txt to use',
                                                                     os.path.abspath('../data'),
                                                                     'Text Files (*.txt)')
        else:
            input_classes_filepath = '../data/label_list/coco_classes.txt'
            ok = True

        if ok:
            self._path_to_label_list = input_classes_filepath
            self._label_list = [line.rstrip('\n') for line in open(input_classes_filepath)]
        else:
            print('No label list set.')
            return
        self.list_button.setStyleSheet('background-color: rgba(0,150,10,255);')
        self._is_labellist_linked = True
        self.validateTraining()

    def setDataset(self):
        '''A function that is triggered by the button labelled, Choose Dataset.'''
        if not self.debug:
            new_filepath_to_dataset = (QFileDialog.
                                       getExistingDirectory(self,
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
            self.dataset_button.setStyleSheet('background-color: rgba(0,200,10,255);')
        else:
            # Set button color to red
            print('Dataset path does not exist.')
            self.dataset_button.setStyleSheet('background-color: red;')

        self.validateTraining()

    def runLabelme(self):
        '''A function that is triggered by the button labelled, Label Dataset.'''
        self.label_process = subprocess.Popen(['labelme'])
        self.validateTraining()

    def initModel(self):
        '''
        A Mutator function that sets the model_name to the first model available
        whenever the precision level changes.
        '''
        self.model_name = self._model_list[0]
        self._is_model_ready = True
        print('Set Model to ', self.model_name)

    def validateTraining(self):
        '''
        A Mutator function that evaulates necessary boolean flags that are all
        required to allow proper training given a certain requested precision level.
        '''
        # Perform 4 checks to ensure all data is available for Training to start without issue.

        if not self._is_model_ready:
            print('No model provided. Please choose Model.')
            self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return
        if not self._is_dataset_linked:
            print('Dataset directory not provided. Please choose Dataset.')
            self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return
        if not self._is_labellist_linked:
            print('Label List not provided. Please choose Label List.')
            self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        if not self._is_dataset_labelled:
            print('Dataset not properly restructured. Please restructure Dataset.')
            self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        # Precision Level 1 only requires 4 checks.
        if self._precision_level == 1:
            print('Precision 1 Training Ready.')
            self.train_button.setStyleSheet('background-color: white;')
            self.connectTrainingButton()
            return

        if not self._is_dataset_labelled:
            print('Dataset not labelled properly. Please label Dataset.')
            self.train_button.setStyleSheet('background-color: rgba(180,180,180,255);')
            self.disconnectTrainingButton()
            return

        self.train_button.setStyleSheet('background-color: white;')
        self.connectTrainingButton()

    def validateDataset(self):
        '''A function that is triggered by the button labelled, Validate Dataset.'''
        if self._precision_level == 1:
            trainDirExists = os.path.exists(self._path_to_dataset + '/train')
            valDirExists = os.path.exists(self._path_to_dataset + '/val')
            # Check if the dataset folder has the following structure
            if trainDirExists and valDirExists:
                self._is_dataset_labelled = True
                self.validate_button.setStyleSheet('background-color: rgba(0,200,10,255);')
            else:
                self._is_dataset_labelled = False
                print('[ERROR] - Please ensure there is ' +
                       '/train and /val sub-directories ' +
                       'in the selected dataset directory.')
        elif self._precision_level == 2:
            isDatasetNamedRight = os.path.basename(self._path_to_dataset) == 'custom_dataset'
            trainDirExists = os.path.exists(self._path_to_dataset + '/train_dataset')
            valDirExists = os.path.exists(self._path_to_dataset + '/val_dataset')
            # Check if the dataset folder has the following structure
            if trainDirExists and valDirExists and isDatasetNamedRight:
                self._is_dataset_labelled = True
                self.validate_button.setStyleSheet('background-color: rgba(0,200,10,255);')
            else:
                self._is_dataset_labelled = False
                print('[ERROR] - Please ensure there is ' +
                      '/train_dataset and /val_dataset sub-directories' +
                      'in the selected dataset directory.')
        elif self._precision_level == 3:
            isDatasetNamedRight = os.path.basename(self._path_to_dataset) == 'custom_dataset'
            trainDirExists = os.path.exists(self._path_to_dataset + '/train_dataset')
            valDirExists = os.path.exists(self._path_to_dataset + '/val_dataset')
            # Check if the dataset folder has the following structure
            if trainDirExists and valDirExists and isDatasetNamedRight:
                self._is_dataset_labelled = True
                self.validate_button.setStyleSheet('background-color: rgba(0,200,10,255);')
            else:
                self._is_dataset_labelled = False
                print('[ERROR] - Please ensure there is ' +
                      '/train_dataset and /val_dataset sub-directories ' +
                      'in the selected dataset directory.')

        self.validateTraining()

    def startTraining(self):
        '''A function that is triggered by the button labelled, Train.'''
        self.disconnectTrainingButton()
        self.train_button.setText('Training In Progress')
        self.train_button.updateGeometry()

        if self._precision_level == 1:
            p1_trainer = P1Trainer(self._path_to_dataset,
                                   self.model_name,
                                   self._label_list)
            p1_trainer.train(False)
        elif self._precision_level == 2:
            p2_trainer = P2Trainer(self._path_to_dataset,
                                   self.model_name,
                                   self._label_list)
            p2_trainer.train(False)
        else:
            p3_trainer = P3Trainer(self._path_to_dataset,
                                   self.model_name,
                                   self._label_list)
            p3_trainer.train(False)

        self.train_button.setText('Train')
        self.train_button.updateGeometry()

    def conformDatasetToCOCO(self):
        '''A function that is triggered by the button labelled, Generate Dataset.'''
        if not self.debug:
            path_to_labelled = QFileDialog.getExistingDirectory(self,
                                                                'Select your labeled dataset.',
                                                                os.path.abspath('../data'),
                                                                QFileDialog.ShowDirsOnly
                                                                | QFileDialog.DontResolveSymlinks)
        else:
            path_to_labelled = '../data/datasets/p2p3_dummy_dataset'
        # Check if every image in given folder
        trainDirExists = os.path.exists(path_to_labelled + '/train_dataset')
        valDirExists = os.path.exists(path_to_labelled + '/val_dataset')

        outputTrainDir = '../data/datasets/custom_dataset/train_dataset'
        outputValDir = '../data/datasets/custom_dataset/val_dataset'

        if trainDirExists and valDirExists:
            self.label_train_process = subprocess.Popen(['python',
                                                         'dataset/labelme2coco.py',
                                                         '--labels',
                                                         self._path_to_label_list,
                                                         path_to_labelled + '/train_dataset',
                                                         outputTrainDir])
            if not self.debug:
                self.label_train_process.communicate()
            self.label_val_process = subprocess.Popen(['python',
                                                       'dataset/labelme2coco.py',
                                                       '--labels',
                                                       self._path_to_label_list,
                                                       path_to_labelled + '/val_dataset',
                                                       outputValDir])
            if not self.debug:
                self.label_val_process.communicate()
        else:
            print('Faulty labelled dataset detected.')

    def populateModelSelector(self):
        '''
        A Mutator function that populates the DropDown Menu labelled, Choose
        Model with all available pretrained models from PyTorch model zoo.
        '''
        # Implement different model list based on different precision level.
        if self._precision_level == 1:
            self._model_list = [line.rstrip('\n') for line in open('./lists/p1_model_list.txt')]
        elif self._precision_level == 2:
            self._model_list = [line.rstrip('\n') for line in open('./lists/p2_model_list.txt')]
        elif self._precision_level == 3:
            self._model_list = [line.rstrip('\n') for line in open('./lists/p3_model_list.txt')]

        self.model_selector.clear()
        for model in self._model_list:
            self.model_selector.addItem(model)

    def connectTrainingButton(self):
        '''
        A Mutator function that allows the button labelled, Train, to be used by
        the user.
        '''
        if not self.buttonConnected:
            self.train_button.clicked.connect(self.startTraining)
            self.buttonConnected = True

    def disconnectTrainingButton(self):
        '''
        A Mutator function that disallows the button labelled, Train, to be used by
        the user.
        '''
        if self.buttonConnected:
            try:
                self.train_button.clicked.disconnect(self.startTraining)
            except Exception:
                pass
            self.buttonConnected = False
