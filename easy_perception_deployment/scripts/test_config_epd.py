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
import json
import pytest
import subprocess

from cli.config_epd import EPDConfigurator

# Change directory to root of easy_perception_deployment ROS2 package.
os.chdir(r"../")
REQUIRED_START_DIR = os.getcwd()

# Reset session_config.json and usecase_config.json to default.
if (os.path.exists('./config/session_config.json') and
   os.path.exists('./config/usecase_config.json')):
    p1 = subprocess.Popen(['rm', './config/session_config.json'])
    p1.communicate()
    p2 = subprocess.Popen(['rm', './config/usecase_config.json'])
    p2.communicate()

    dict = {
        "path_to_model": './data/model/squeezenet1.1-7.onnx',
        "path_to_label_list": './data/label_list/imagenet_classes.txt',
        "visualizeFlag": 'visualize',
        "useCPU": 'CPU'
        }
    json_object = json.dumps(dict, indent=4)
    with open('./config/session_config.json', 'w') as outfile:
        outfile.write(json_object)

    dict = {"usecase_mode": 0}
    json_object = json.dumps(dict, indent=4)
    with open('./config/usecase_config.json', 'w') as outfile:
        outfile.write(json_object)


def test_invalid_ExeDirectory():
    # Change to invalid directory.
    os.chdir(r"./scripts")
    test_args = ['scripts/config_epd.py', '-v']
    INVALID_START_DIR = os.getcwd()

    with pytest.raises(SystemExit) as pytest_wrapped_e:
        configurator = EPDConfigurator(INVALID_START_DIR, test_args)

    # Check for sys.exit() due to invalid_start_dir
    assert pytest_wrapped_e.type == SystemExit
    os.chdir(r"../")


def test_print_help_NoArgs(capfd):
    test_args = ['scripts/config_epd.py']

    with pytest.raises(SystemExit) as pytest_wrapped_e:
        configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    assert pytest_wrapped_e.type == SystemExit


def test_print_help_HelpArg(capfd):
    test_args = ['scripts/config_epd.py', '-h']

    with pytest.raises(SystemExit) as pytest_wrapped_e:
        configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    assert pytest_wrapped_e.type == SystemExit


def test_set_VisualizeMode_short():

    test_args = ['scripts/config_epd.py', '-v']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    visualizeFlag = data["visualizeFlag"]
    f.close()

    assert visualizeFlag == "visualize"


def test_set_VisualizeMode_long():

    test_args = ['scripts/config_epd.py', '--visualize']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    visualizeFlag = data["visualizeFlag"]
    f.close()

    assert visualizeFlag == "visualize"


def test_set_ActionMode_short():

    test_args = ['scripts/config_epd.py', '-a']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    visualizeFlag = data["visualizeFlag"]
    f.close()

    assert visualizeFlag == "robot"


def test_set_ActionMode_long():

    test_args = ['scripts/config_epd.py', '--action']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    visualizeFlag = data["visualizeFlag"]
    f.close()

    assert visualizeFlag == "robot"


def test_set_ActionMode_long():

    test_args = ['scripts/config_epd.py', '--action']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    visualizeFlag = data["visualizeFlag"]
    f.close()

    assert visualizeFlag == "robot"


def test_set_GPU_short():

    test_args = ['scripts/config_epd.py', '-g']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    useCPU = data["useCPU"]
    f.close()

    assert useCPU == "GPU"


def test_set_GPU_long():

    test_args = ['scripts/config_epd.py', '--gpu']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    useCPU = data["useCPU"]
    f.close()

    assert useCPU == "GPU"


def test_set_CPU_short():

    test_args = ['scripts/config_epd.py', '-c']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    useCPU = data["useCPU"]
    f.close()

    assert useCPU == "CPU"


def test_set_CPU_long():

    test_args = ['scripts/config_epd.py', '--cpu']
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    useCPU = data["useCPU"]
    f.close()

    assert useCPU == "CPU"


def test_set_ValidModel():

    # Create dummy model file in /data
    PATH_TO_DUMMY_MODEL = './data/model/DUMMY_MODEL.onnx'
    p1 = subprocess.Popen(['touch', PATH_TO_DUMMY_MODEL])
    p1.communicate()

    test_args = ['scripts/config_epd.py', '--model', PATH_TO_DUMMY_MODEL]
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    model_path = data["path_to_model"]
    f.close()

    assert model_path == PATH_TO_DUMMY_MODEL

    # Remove dummy model file in /data
    p1 = subprocess.Popen(['rm', PATH_TO_DUMMY_MODEL])
    p1.communicate()


def test_set_InvalidModel():

    PATH_TO_INVALID_MODEL = './data/model/NONEXISTENT_MODEL.onnx'

    test_args = ['scripts/config_epd.py', '--model', PATH_TO_INVALID_MODEL]
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    with pytest.raises(SystemExit) as pytest_wrapped_e:
        configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    assert pytest_wrapped_e.type == SystemExit


def test_set_ValidLabelList():

    PATH_TO_VALID_LABEL_LIST = './data/label_list/coco_classes.txt'

    test_args = ['scripts/config_epd.py', '--label', PATH_TO_VALID_LABEL_LIST]
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    # Load session_config.json
    f = open(session_config_filepath)
    data = json.load(f)
    label_list_path = data["path_to_label_list"]
    f.close()

    assert label_list_path == PATH_TO_VALID_LABEL_LIST


def test_set_InvalidLabelList():

    PATH_TO_INVALID_LABEL_LIST = './data/label_list/NONEXISTENT_LABEL_LIST.txt'

    test_args = [
        'scripts/config_epd.py',
        '--label',
        PATH_TO_INVALID_LABEL_LIST]
    session_config_filepath = REQUIRED_START_DIR \
        + "/config/session_config.json"

    with pytest.raises(SystemExit) as pytest_wrapped_e:
        configurator = EPDConfigurator(REQUIRED_START_DIR, test_args)

    assert pytest_wrapped_e.type == SystemExit
