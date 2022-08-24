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

from PySide2.QtCore import QSize
from PySide2.QtGui import QIcon
from PySide2.QtWidgets import QPushButton, QWidget

from windows.Deploy import DeployWindow
from windows.Train import TrainWindow

import logging
from datetime import datetime


class MainWindow(QWidget):
    '''
    The MainWindow class is a PySide2 Graphical User Interface (GUI) window
    that starts up as the first user interface.
    '''
    def __init__(self):
        '''
        The constructor.
        Sets the size of the window.
        Calls setButtons function to populate window with button.
        '''
        super().__init__()

        timestamp = datetime.now()
        timestamp_string = timestamp.strftime("%d-%m-%Y-%H-%M-%S")

        logging.basicConfig(
            level=logging.NOTSET,
            format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
            datefmt='%m-%d %H:%M',
            filename='log/' + timestamp_string + '.log',
            filemode='w')
        warn_console = logging.StreamHandler()
        warn_console.setLevel(logging.WARN)
        info_console = logging.StreamHandler()
        info_console.setLevel(logging.INFO)
        error_console = logging.StreamHandler()
        error_console.setLevel(logging.ERROR)
        formatter = logging.Formatter(
            '%(name)-12s: ' +
            '%(levelname)-8s %(message)s')
        warn_console.setFormatter(formatter)
        info_console.setFormatter(formatter)
        error_console.setFormatter(formatter)
        logging.getLogger('').addHandler(warn_console)
        logging.getLogger('').addHandler(info_console)
        logging.getLogger('').addHandler(error_console)

        self.train_window = TrainWindow(False)
        self.deploy_window = DeployWindow(False)
        self.isTrainOpen = False
        self.isDeployOpen = False

        self._WINDOW_HEIGHT = 375
        self._WINDOW_WIDTH = 500

        self.setWindowIcon(QIcon("img/epd_desktop.png"))

        self.setWindowTitle('easy_perception_deployment')
        self.setGeometry(0, 0, self._WINDOW_WIDTH, self._WINDOW_HEIGHT)

        self.setButtons()

    def setButtons(self):
        '''A Mutator function that defines all buttons in MainWindow.'''
        self.train_button = QPushButton('Train', self)
        self.train_button.setIcon(QIcon('img/train.png'))
        self.train_button.setIconSize(QSize(100, 100))
        self.train_button.setGeometry(0, 0, self._WINDOW_WIDTH/2, 250)

        self.deploy_button = QPushButton('Deploy', self)
        self.deploy_button.setIcon(QIcon('img/deploy.png'))
        self.deploy_button.setIconSize(QSize(100, 100))
        self.deploy_button.setGeometry(self._WINDOW_WIDTH/2,
                                       0,
                                       self._WINDOW_WIDTH/2,
                                       250)

        self.quit_button = QPushButton('Quit', self)
        self.quit_button.setIcon(QIcon('img/quit.png'))
        self.quit_button.setIconSize(QSize(250, 250))
        self.quit_button.setGeometry(0, 250, self._WINDOW_WIDTH, 125)

        self.train_button.clicked.connect(self.openTrainWindow)
        self.deploy_button.clicked.connect(self.deployPackage)
        self.quit_button.clicked.connect(self.closeWindow)

    def deployPackage(self):
        '''A function that is triggered by the button labelled, Deploy.'''
        # Start Deploy window that allows you to set the
        self.isDeployOpen = not self.isDeployOpen

        if (self.isDeployOpen):
            self.deploy_window.show()
        else:
            self.deploy_window.close()

    def openTrainWindow(self):
        '''A function that is triggered by the button labelled, Train.'''
        self.isTrainOpen = not self.isTrainOpen

        if (self.isTrainOpen):
            self.train_window.show()
        else:
            self.train_window.close()

    def closeWindow(self):
        '''A function that is triggered by the button labelled, Quit.'''
        self.close()
        self.train_window.close()
        self.deploy_window.close()
