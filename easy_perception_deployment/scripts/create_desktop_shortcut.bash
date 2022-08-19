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

# Function: Create desktop shortcut to EPD on the Desktop.
# Static Analysis: shellcheck create_desktop_shortcut.bash

path_to_gui_run_script="$PWD/run.bash"
path_to_gui_icon_file="$PWD/gui/img/epd_desktop.png"

outputdir1="$HOME/Desktop/"
outputdir2="$HOME/.local/share/applications/"

if [ -f "$outputdir1/easy_perception_deployment.desktop" ]; then
    echo "Desktop shortcut has already generated on your Desktop. Please delete it before running this script again."
    exit 1
fi

if [ -f "$path_to_gui_run_script" ]; then
    touch easy_perception_deployment.desktop
    {
      echo "[Desktop Entry]"
      echo "Name=easy_perception_deployment"
      echo "Comment=A ROS2 package that accelerates the training and deployment of CV models in industries."
      echo "Exec=$path_to_gui_run_script"
      echo "Icon=$path_to_gui_icon_file"
      echo "Terminal=true"
      echo "Type=Application"
      echo "Categories=Application;"
    } > easy_perception_deployment.desktop
    sudo chmod u+x easy_perception_deployment.desktop
    cp easy_perception_deployment.desktop "$outputdir1"
    cp easy_perception_deployment.desktop "$outputdir2"
    rm easy_perception_deployment.desktop
else
    echo "[-ERROR-]: Please run this script in [-easy_perception_deployment-] package directory."
    exit 1
fi

unset path_to_gui_py_file outputdir1 outputdir2 path_to_gui_run_script path_to_gui_icon_file
