#!/usr/bin/env bash

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
import argparse
import getopt

def print_help():
    print('config_epd.py [--visualize ] [--action ] [--cpu ] [--gpu ] ' +
        '[--model ] [--label ]')
    print()
    print('-v --visualize   Sets EPD to Visualize Mode.')
    print('-a --action   Sets EPD to Action Mode.')
    print('-g --gpu   Sets EPD to GPU Mode.')
    print('-c --cpu   Sets EPD to CPU Mode.')
    print('--model   Sets new onnx model to be deployed via EPD.')
    print('--label   Sets new label list to be deployed via EPD.')


class EPDConfigurator():

    def __init__(self, start_dirpath, args):

        self._path_to_model = ''
        self._path_to_label_list = ''
        self._input_image_topic = ''
        self.visualizeFlag = True
        self.useCPU = True
        self.isService = False

        self.usecase_mode = 0

        self.session_config = []
        self.usecase_config = []

        # Check if session_config.txt exits.
        self.session_config_filepath = start_dirpath + "/config/session_config.json"
        if os.path.isfile(self.session_config_filepath):
            print("[ config_epd ] - session_config.json detected.")
            self.parse_session_config(self.session_config_filepath)
        else:
            print("[ config_epd ] - ERROR. session_config.json missing.")
            sys.exit(1)
        # Check if usecase_config.txt exists.
        self.usecase_config_filepath = start_dirpath + "/config/usecase_config.json"
        if os.path.isfile(self.usecase_config_filepath):
            print("[ config_epd ] - usecase_config.txt detected.")
            self.parse_usecase_config(self.usecase_config_filepath)
        else:
            print("[ config_epd ] - ERROR. usecase_config.json missing.")
            sys.exit(1)

        if len(args) < 2:
            print('Please specify a configuration.')
            print_help()
            sys.exit(1)

        opt_files = self.parse_args(args[1:])

        self.write_to_session_config(self.session_config_filepath)


    def parse_args(self, args):
        opts, opt_files = getopt.getopt(args, 'hvagc',
                                        ['visualize',
                                         'action',
                                         'gpu',
                                         'cpu',
                                         'model=',
                                         'label='])

        for opt, arg in opts:
            if opt == '-h':
                print_help()
                sys.exit(0)
            elif opt in ('-v', '--visualize'):
                self.visualizeFlag = True
                self.session_config[2] = "visualize"
            elif opt in ('-a', '--action'):
                self.visualizeFlag = False
                self.session_config[2] = "robot"
            elif opt in ('-g', '--gpu'):
                self.useCPU = False
                self.session_config[3] = "GPU"
            elif opt in ('-c', '--cpu'):
                self.useCPU = True
                self.session_config[3] = "CPU"
            elif opt in ('-m', '--model'):
                if not os.path.isfile(os.getcwd() + "/" + arg):
                    print("[ config_epd ] - ERROR. input model file does not exist.")
                    print("[ config_epd ] - Exiting.")
                    sys.exit(2)
                self._path_to_model = "./" + arg
                self.session_config[0] = "./" + arg
            elif opt in ('-l', '--label'):
                if not os.path.isfile(os.getcwd() + "/" + arg):
                    print("[ config_epd ] - ERROR. input label list does not exist.")
                    print("[ config_epd ] - Exiting.")
                    sys.exit(2)
                self._path_to_label_list = "./" + arg
                self.session_config[1] = "./" + arg

    def parse_session_config(self, session_config_filepath):
        #TODO(cardboardcode): Implement parser for session_config.json.
        print("[ parse_session_config ] - FUNCTION START ")

    def parse_usecase_config(self, usecase_config_filepath):
        #TODO(cardboardcode): Implement parser for usecase_config.json.
        print("[ parse_use_case_config ] - FUNCTION START")

    def write_to_session_config(self, session_config_filepath):
        #TODO(cardboardcode): Implement write out function to session_config.json
        #TODO(cardboardcode): Implement seperate write out function to usecase_config.json

def main(args=None):

    # Checks if this script is run in the root of the easy_perception_deployment ROS2 package.
    start_dirpath = os.getcwd()
    # Check if the following folder are in the directory when script is being executed:
    # data
    # CMakeLists.txt and package.xml

    if os.path.isdir(start_dirpath + "/data"):
        if os.path.isdir(start_dirpath + "/scripts") and os.path.isdir(start_dirpath + "/launch"):
            print("[ config_epd ] - Executing in root of EPD package.")
            configurator = EPDConfigurator(start_dirpath, args)
        else:
            print("[ config_epd ] - ERROR. /scripts or /launch is missing.")
            print("[ config_epd ] - Exiting...")
            sys.exit(1)
    else:
        print("[ config_epd ] - ERROR. data folder is missing.")
        print("[ config_epd ] - Exiting...")
        sys.exit(1)

if __name__ == "__main__":
    main(sys.argv)
