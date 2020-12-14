#!/usr/bin/env bash

# reference: https://github.com/microsoft/onnxruntime#installation
WORKDIR=/tmp/onnxruntime
mkdir -p ${WORKDIR}

apt-get install -y git

env "PATH=$PATH"

readonly CURRENT_DIR=$(dirname $(realpath $0))
readonly CMAKE_LOWEST_VERSION="3.13"

function command_exists
{
    type "$1" &> /dev/null;
}

if ! command_exists cmake; then
    echo "need cmake ${CMAKE_LOWEST_VERSION} or above"
fi

readonly CMAKE_VERSION=`cmake --version | head -n1 | cut -d" " -f3`

function version_greater_equal
{
    printf '%s\n%s\n' "$2" "$1" | sort -V -C
}

# if ! version_greater_equal "${CMAKE_VERSION}" "${CMAKE_LOWEST_VERSION}"; then
#     echo "need cmake ${CMAKE_LOWEST_VERSION} or above"
#     echo "Installing CMake 3.15.2"
#     readonly CMAKE_VERSION_TO_INSTALL="3.15.2"
#
#     # apt-get purge -y cmake
#     apt-get install -y wget
#
#     cd /tmp
#     wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION_TO_INSTALL}/cmake-${CMAKE_VERSION_TO_INSTALL}.tar.gz
#     tar -zxvf cmake-${CMAKE_VERSION_TO_INSTALL}.tar.gz
#     cd cmake-${CMAKE_VERSION_TO_INSTALL}
#     ./bootstrap
#     make install
# fi

apt-get install -y libgomp1
apt-get install -y zlib1g-dev
apt-get install -y locales
apt-get install -y language-pack-en
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8

echo "-------------------------------------------------------------------------"
echo "cloning onnxruntime and starting to build..."
echo "-------------------------------------------------------------------------"

readonly ONNXRUNTIME_VERSION="v1.3.1"

git clone --recursive https://github.com/Microsoft/onnxruntime ${WORKDIR}
cd ${WORKDIR}

readonly INSTALL_PREFIX="/usr/local"
BUILDTYPE=Release
BUILDARGS="--config ${BUILDTYPE}"
# BUILDARGS="${BUILDARGS} --use_cuda --cuda_version=${CUDA_VERSION} --cuda_home=${CUDA_HOME} --cudnn_home=${CUDNN_HOME}"
BUILDARGS="${BUILDARGS} --parallel"
BUILDARGS="${BUILDARGS} --update"
BUILDARGS="${BUILDARGS} --use_openmp"
# BUILDARGS="${BUILDARGS} --use_tensorrt --tensorrt_home /usr/src/tensorrt/"
BUILDARGS="${BUILDARGS} --cmake_extra_defines CMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}"

# options to preserve environment path of current user
env "PATH=$PATH" sh build.sh ${BUILDARGS} --build
env "PATH=$PATH" sh build.sh ${BUILDARGS} --build_shared_lib

cd ./build/Linux/${BUILDTYPE}
make install
