#!/usr/bin/env bash

path_to_gui_run_script="$PWD/run.bash"
path_to_gui_icon_file="$PWD/gui/img/gui.png"

outputdir1="$HOME/Desktop/"
outputdir2="$HOME/.local/share/applications/"

if test -f "$outputdir"; then
    echo "Desktop shortcut has already generated on your Desktop. Please delete it before running this script again."
    exit 1
fi

if test -f "$path_to_gui_run_script"; then
    touch easy_perception_deployment.desktop
    echo "[Desktop Entry]" >> easy_perception_deployment.desktop
    echo "Name=easy_perception_deployment" >> easy_perception_deployment.desktop
    echo "Comment=A ROS2 package that accelerates the training and deployment of CV models in industries." >> easy_perception_deployment.desktop
    echo "Exec=$path_to_gui_run_script" >> easy_perception_deployment.desktop
    echo "Icon=$path_to_gui_icon_file" >> easy_perception_deployment.desktop
    echo "Terminal=true" >> easy_perception_deployment.desktop
    echo "Type=Application" >> easy_perception_deployment.desktop
    echo "Categories=Application;" >> easy_perception_deployment.desktop
    sudo chmod u+x easy_perception_deployment.desktop
    cp easy_perception_deployment.desktop $outputdir1
    cp easy_perception_deployment.desktop $outputdir2
    rm easy_perception_deployment.desktop
else
    echo "[-ERROR-]: Please run this script in [-easy_perception_deployment-] package directory."
    exit 1
fi

unset path_to_gui_py_file
