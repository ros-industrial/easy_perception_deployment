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

# Function: Installs all dependencies required for EPD to function on CPU mode.
# Static Analysis: shellcheck install_dep_cpu.bash -e SC2086
# Reference: https://github.com/microsoft/onnxruntime#installation

# Checks if a command exit
function command_exists() {
    type "$1" &> /dev/null;
}

# Compares two CMake versions.
function version_greater_equal() {
    printf '%s\n%s\n' "$2" "$1" | sort -V -C
}

readonly CMAKE_LOWEST_VERSION="3.13"
readonly CMAKE_VERSION=$(cmake --version | head -n1 | cut -d" " -f3)

# Check if wifi is on. This is to prevent early detection and
# termination of faulty download procedure before any download is done.
if wget -q --spider http://google.com; then
    echo "[ WIFI ] - FOUND . Proceeding with download."
else
    echo "[ WIFI ] - NOTFOUND . Please ensure you are properly connected to the internet before running this script again."
    exit 1
fi

sudo -l env "PATH=$PATH"

echo "-------------------------------------------------------------------------"
echo "Checking dependencies..."
echo "-------------------------------------------------------------------------"

if ! command_exists cmake; then
    echo "Require CMake ${CMAKE_LOWEST_VERSION} or above"
fi

# Checks if the current CMake version is lower than the lowest permissible
# CMake version 3.13
# If so, install CMake 3.15.2
if ! version_greater_equal "${CMAKE_VERSION}" "${CMAKE_LOWEST_VERSION}"; then
    echo "Require CMake ${CMAKE_LOWEST_VERSION} or above"

    readonly VERSION="3.15.2"
    cd "$HOME" || exit
    sudo apt-get install -y wget
    wget https://github.com/Kitware/CMake/releases/download/v${VERSION}/cmake-${VERSION}.tar.gz
    tar -zxvf cmake-${VERSION}.tar.gz
    cd cmake-${VERSION} || exit
    ./bootstrap
    make install
    export PATH=$HOME/cmake-3.15.2/bin:$PATH
    export CMAKE_PREFIX_PATH=$HOME/cmake-3.15.2:$CMAKE_PREFIX_PATH
fi

sudo apt-get install -y wget
# Install colcon tools that don't come default with ROS2.
sudo apt-get install -y python3-colcon-common-extensions
# Install git and specific rosdep
sudo apt-get install -y git python3-rosdep
# Install onnxruntime dependencoes
sudo apt-get install -y libgomp1
sudo apt-get install -y zlib1g-dev
sudo apt-get install -y locales
sudo apt-get install -y language-pack-en
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

echo "-------------------------------------------------------------------------"
echo "Installing Anaconda3..."
echo "-------------------------------------------------------------------------"

# Checks if conda is installed.
# If so, skip.
# Otherwise, install Anaconda.
if conda --version; then
    :
else
    cd "$HOME" || exit
    wget https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh
    bash Anaconda3-2020.07-Linux-x86_64.sh -b -p "/home/$USER/anaconda3"
    rm Anaconda3-2020.07-Linux-x86_64.sh
    export PATH="/home/$USER/anaconda3/bin:$PATH"
    conda init
    conda config --set auto_activate_base False
    conda deactivate
fi

echo "-------------------------------------------------------------------------"
echo "Installing onnxruntime..."
echo "-------------------------------------------------------------------------"

cd "$HOME" || exit
# Checks if there is already an existing onnxruntime library installed.
# If there is no existing library, download
if [ ! -d  onnxruntime ]; then
  git clone --recursive https://github.com/Microsoft/onnxruntime
  cd onnxruntime || exit
  git reset --hard 36dc057913f968566eaa1646cb5db41d8c5e7654
else
  echo "Found an existing onnxruntime."
  read -p "Do you wish to overwrite it [y/n]? " -n 1 -r
  echo    # (optional) move to a new line
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      echo "Using existing onnxruntime directory."
      cd onnxruntime || exit
  else
      sudo rm -r onnxruntime
      git clone --recursive https://github.com/Microsoft/onnxruntime
      cd onnxruntime || exit
      git reset --hard 36dc057913f968566eaa1646cb5db41d8c5e7654
  fi
fi

readonly INSTALL_PREFIX="/usr/local"
BUILDTYPE=Release
BUILDARGS="--config ${BUILDTYPE}"
BUILDARGS="${BUILDARGS} --parallel"
BUILDARGS="${BUILDARGS} --update"
BUILDARGS="${BUILDARGS} --use_openmp"
BUILDARGS="${BUILDARGS} --cmake_extra_defines CMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}"

# Set options to preserve environment path of current user
sudo env "PATH=$PATH" ./build.sh ${BUILDARGS} --build
sudo env "PATH=$PATH" ./build.sh ${BUILDARGS} --build_shared_lib

cd "./build/Linux/${BUILDTYPE}" || exit
sudo make install
echo "Uninstall with cat install_manifest.txt | sudo xargs rm"
