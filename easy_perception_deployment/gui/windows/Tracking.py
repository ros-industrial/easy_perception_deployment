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
import json
import logging

from PySide2.QtCore import QSize
from PySide2.QtGui import QIcon
from PySide2.QtWidgets import QComboBox, QLabel
from PySide2.QtWidgets import QMessageBox, QPushButton, QWidget


class TrackingWindow(QWidget):
    '''
    The TrackingWindow class is a PySide2
    Graphical User Interface (GUI) window that is called by
    DeployWindow class in order to configure a custom Tracking
    use-case and write to usecase_config.json.
    '''
    def __init__(self, _path_to_usecase_config):
        '''
        The constructor.
        Sets the size of the window and configurations for usecase_config.
        Checks if the usecase_config.json file exists. If true, configure
        accordingly. Otherwise, assign default values.
        Calls setButtons function to populate window with button.
        '''
        super().__init__()

        self.tracking_logger = logging.getLogger('tracking')

        self._TRACKING_WIN_H = 150
        self._TRACKING_WIN_W = 300

        self.setWindowIcon(QIcon("img/epd_desktop.png"))

        self._label_list = []
        self._select_list = []
        self._path_to_usecase_config = _path_to_usecase_config

        self._selected_tracker_index = 0
        self._tracker_label_list = ['KCF', 'MEDIANFLOW', 'CSRT']
        self._tracker_description_list = [
            'KCF - General Purpose Tracker.',
            'MedianFlow - For objects with predictable movements.',
            'CSRT - For objects with unpredictable movements.']

        self.setWindowTitle('Choose OpenCV Tracker.')
        self.setGeometry(self._TRACKING_WIN_W, self._TRACKING_WIN_H,
                         self._TRACKING_WIN_W, self._TRACKING_WIN_H)
        self.setFixedSize(self._TRACKING_WIN_W, self._TRACKING_WIN_H)

        self.setButtons()

    def setButtons(self):
        '''A Mutator function that defines all buttons in TrackingWindow.'''
        # Label List Menu for showing and adding objects-to-count
        self.label_list_dropdown = QComboBox(self)
        self.label_list_dropdown.setGeometry(0,
                                             0,
                                             self._TRACKING_WIN_W,
                                             50)
        for label in self._tracker_description_list:
            self.label_list_dropdown.addItem(label)

        # Finish button to save the stored counting
        # and write to usecase_config.json
        self.finish_button = QPushButton('Done', self)
        self.finish_button.setIcon(QIcon('img/go.png'))
        self.finish_button.setIconSize(QSize(75, 75))
        self.finish_button.setGeometry(self._TRACKING_WIN_W/2,
                                       50,
                                       self._TRACKING_WIN_W/2,
                                       100)
        # Cancel button to exit USE CASE: TRACKING
        self.cancel_button = QPushButton('Cancel', self)
        self.cancel_button.setIcon(QIcon('img/quit.png'))
        self.cancel_button.setIconSize(QSize(75, 75))
        self.cancel_button.setGeometry(0,
                                       50,
                                       self._TRACKING_WIN_W/2,
                                       100)

        self.finish_button.clicked.connect(self.writeToUseCaseConfig)
        self.cancel_button.clicked.connect(self.closeWindow)
        self.label_list_dropdown.activated.connect(self.selectTracker)

    def writeToUseCaseConfig(self):
        '''A function that is triggered by the button labelled, Finish.'''
        self.tracking_logger.info('Wrote to ../data/usecase_config.json')

        dict = {
            "usecase_mode": 4,
            "track_type": (self._tracker_label_list[
                self._selected_tracker_index])
            }
        json_object = json.dumps(dict, indent=4)
        with open(self._path_to_usecase_config, 'w') as outfile:
            outfile.write(json_object)

    def closeWindow(self):
        '''A function that is triggered by the button labelled, Cancel.'''
        self.close()

    def selectTracker(self, index):
        '''
        A function that is triggered by the DropDown Menu labelled, Available
        Trackers
        '''
        self.tracking_logger.info(self._tracker_label_list[index] +
                                  ' tracker chosen.')

        self._selected_tracker_index = index
