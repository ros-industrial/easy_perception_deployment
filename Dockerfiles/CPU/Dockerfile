FROM ros:foxy-ros-base-focal

MAINTAINER Bey Hao Yun <beyhy@artc.a-star.edu.sg>
ENV DEBIAN_FRONTEND noninteractive

# Add user
RUN adduser --quiet --disabled-password epduser

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
    BUILDTYPE=Release  && \
    set -- --config "${BUILDTYPE}" \
           --parallel \
           --update \
           --use_openmp \
           --cmake_extra_defines CMAKE_INSTALL_PREFIX="/usr/local" && \
    env "PATH=$PATH" sh build.sh "$@" --build  && \
    env "PATH=$PATH" sh build.sh "$@" --build_shared_lib  && \
    make install -C ./build/Linux/"${BUILDTYPE}" && \
    rm -rf ./*

# Install OpenCV, libboost-dev, ros-foxy-cv-bridge, ros-foxy-pcl-conversions
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y libboost-dev && \
    apt-get install -y ros-foxy-cv-bridge && \
    apt-get install -y ros-foxy-pcl-conversions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /
