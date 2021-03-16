FROM vistart/cuda:10.2-cudnn7-devel-ubuntu20.04

MAINTAINER Bey Hao Yun <beyhy@artc.a-star.edu.sg>

CMD nvidia-smi

# Set up timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Set up keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Set up sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# Set up environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO foxy

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-core=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Set up colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1*  && \
    rm -rf /var/lib/apt/lists/*

# Install gcc 8.4.0 and make default
RUN apt-get update && apt-get install -y gcc-8 g++-8 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 70 --slave /usr/bin/g++ g++ /usr/bin/g++-9 --slave /usr/bin/gcov gcov /usr/bin/gcov-9 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 80 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8 && \
    update-alternatives --auto gcc

WORKDIR /tmp/onnxruntime

# Install onnxruntime from source
RUN git clone https://github.com/Microsoft/onnxruntime . \
              --recursive && \
    git reset --hard 16d35266ab9e060c5866a0cb629e37b50015130e && \
    mkdir -p include/onnxruntime/core/flatbuffers && \
    CUDA_HOME=/usr/local/cuda && \
    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64 && \
    PATH=$PATH:${CUDA_HOME}/bin && \
    CUDA_VERSION="10." && \
    CUDNN_HOME=$(dirname $(dpkg -L libcudnn7 | grep libcudnn.so | head -n1)) && \
    BUILDTYPE=Release  && \
    set -- --config "${BUILDTYPE}" \
           --use_cuda \
           --cuda_version=${CUDA_VERSION} \
           --cuda_home=${CUDA_HOME} \
           --cudnn_home=${CUDNN_HOME} \
           --parallel \
           --update \
           --use_openmp \
           --cmake_extra_defines CMAKE_INSTALL_PREFIX="/usr/local" && \
    env "PATH=$PATH" sh build.sh "$@" --build  && \
    env "PATH=$PATH" sh build.sh "$@" --build_shared_lib  && \
    make install -C ./build/Linux/"${BUILDTYPE}" && \
    rm -rf ./*

# Install OpenCV, libboost-dev, ros-foxy-cv-bridge libpcl-dev
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y libboost-dev && \
    apt-get install -y ros-foxy-cv-bridge && \
    apt-get install -y ros-foxy-pcl-conversions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /
