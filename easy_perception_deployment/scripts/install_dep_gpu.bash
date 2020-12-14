#!/usr/bin/env bash

# reference: https://github.com/microsoft/onnxruntime#installation

sudo -l env "PATH=$PATH"

export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin

readonly CURRENT_DIR=$(dirname $(realpath $0))

echo "-------------------------------------------------------------------------"
echo "Checking dependencies..."
echo "-------------------------------------------------------------------------"

readonly CMAKE_LOWEST_VERSION="3.13"

function command_exists
{
    type "$1" &> /dev/null;
}

if ! command_exists cmake; then
    echo "Require CMake ${CMAKE_LOWEST_VERSION} or above"
fi

if ! command_exists nvcc; then
    echo "Need to install cuda package"
    exit
fi

readonly CMAKE_VERSION=`cmake --version | head -n1 | cut -d" " -f3`
readonly CUDA_VERSION=`nvcc --version | grep release | awk '{print $6}' | cut -c 2-4`
function version_greater_equal
{
    printf '%s\n%s\n' "$2" "$1" | sort -V -C
}

if ! version_greater_equal "${CMAKE_VERSION}" "${CMAKE_LOWEST_VERSION}"; then
    echo "Require CMake ${CMAKE_LOWEST_VERSION} or above"
    readonly VERSION="3.15.2"
    cd $HOME
    sudo apt-get install -y wget

    wget https://github.com/Kitware/CMake/releases/download/v${VERSION}/cmake-${VERSION}.tar.gz
    tar -zxvf cmake-${VERSION}.tar.gz
    cd cmake-${VERSION}
    ./bootstrap
    make install
    export PATH=$HOME/cmake-3.15.2/bin:$PATH
    export CMAKE_PREFIX_PATH=$HOME/cmake-3.15.2:$CMAKE_PREFIX_PATH
fi

readonly CUDA_HOME=/usr/local/cuda

if dpkg -L libcudnn7 &> /dev/null; then
    readonly CUDNN_HOME=`dirname $(dpkg -L libcudnn7 | grep libcudnn.so | head -n1)`
elif ls ${CUDA_HOME}/include/ | grep cudnn &> /dev/null; then
    readonly CUDNN_HOME=${CUDA_HOME}
else
    "Need to install cudnn"
    exit
fi

sudo apt-get install -y wget
# Install colcon tools that don't come default with ROS2.
sudo apt-get install -y python3-colcon-common-extensions
# Install git and specific rosdep
sudo apt-get install -y git python3-rosdep
# Install onnxruntime dependencoes
sudo apt-get install -y libgomp1
sudo apt install zlib1g-dev
sudo apt-get install -y locales
sudo apt-get install -y language-pack-en
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

echo "-------------------------------------------------------------------------"
echo "Installing Anaconda3..."
echo "-------------------------------------------------------------------------"

if output=$(conda --version); then
    :
else
    cd $HOME
    wget https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh
    bash Anaconda3-2020.07-Linux-x86_64.sh -b -p /home/$USER/anaconda3
    rm Anaconda3-2020.07-Linux-x86_64.sh
    export PATH="/home/$USER/anaconda3/bin:$PATH"
    conda init
    conda config --set auto_activate_base False
    conda deactivate
fi

unset output

echo "-------------------------------------------------------------------------"
echo "Installing onnxruntime..."
echo "-------------------------------------------------------------------------"

readonly ONNXRUNTIME_VERSION="v1.3.1"

cd $HOME
git clone --recursive https://github.com/Microsoft/onnxruntime
cd onnxruntime

readonly INSTALL_PREFIX="/usr/local"
BUILDTYPE=Release
BUILDARGS="--config ${BUILDTYPE}"
BUILDARGS="${BUILDARGS} --use_cuda --cuda_version=${CUDA_VERSION} --cuda_home=${CUDA_HOME} --cudnn_home=${CUDNN_HOME}"
BUILDARGS="${BUILDARGS} --parallel"
BUILDARGS="${BUILDARGS} --update"
BUILDARGS="${BUILDARGS} --use_openmp"
# BUILDARGS="${BUILDARGS} --use_tensorrt --tensorrt_home /usr/src/tensorrt/"
BUILDARGS="${BUILDARGS} --cmake_extra_defines CMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}"

# options to preserve environment path of current user
sudo env "PATH=$PATH" ./build.sh ${BUILDARGS} --build
sudo env "PATH=$PATH" ./build.sh ${BUILDARGS} --build_shared_lib

cd ./build/Linux/${BUILDTYPE}
sudo make install
echo "Uninstall with cat install_manifest.txt | sudo xargs rm"
