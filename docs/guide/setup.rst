.. _guide_setup:

Setup
=====

To get started with **easy_perception_deployment**, there are two ways to set it up for use.
1. Docker Installation
2. Local Installation

For a relative painless setup experience, the **Docker Installation** is recommended.

For better system performance, the **Local Installation** is recommended.

System Requirements for Docker Installation
+++++++++++++++++++++++++++++++++++++++++++
1. `Anaconda3 <https://docs.anaconda.com/anaconda/install/>`_
2. `nvidia docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian>`_ - Follow the linked instructions to install this dependency.


Docker Installation
+++++++++++++++++++
Follow the instructions below to start building the **easy_perception_deployment**
ROS2 package using Docker.

1. Generate **desktop shortcut**. A ``.desktop`` file should appear in your Desktop.

.. code-block:: bash

   cd ~/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment/
   sudo chmod u+x scripts/create_desktop_shortcut.bash
   ./scripts/create_desktop_shortcut.bash

2. Follow the deployment instructions under section, Deploying a custom ONNX Model.

The docker images and container will be built and run automatically once the **Run** button is clicked.

This process takes a long time so be prepared to wait while the program takes care of all the installation trouble.

System Requirements for Local Installation
++++++++++++++++++++++++++++++++++++++++++
1. **Ubuntu 20.04** (Focal Fossa) Operating System.
2. `ROS2 Foxy Fitzroy <https://index.ros.org/doc/ros2/Installation/Foxy/>`_
3. `Anaconda3 <https://docs.anaconda.com/anaconda/install/>`_
4. `ONNX Runtime <https://microsoft.github.io/onnxruntime/>`_ - Follow `Installation`_ below to install this dependency.
5. CMake >=3.15.2


*[Optional for Higher Performance]*

1. CUDA (System tested on CUDA 10.2)
2. CUDNN (System tested on CUDNN 7.6.5)


Local Installation
++++++++++++++++++
Follow the instructions below to start building the **easy_perception_deployment**
ROS2 package locally.

The following is the expected final folder structure by following the steps.

.. code-block:: bash

   ~/epd_ros2_ws/
    |_src
      |_easy_perception_deployment
        |_easy_perception_deployment
          |_<package_content ...>
        |_epd_msgs
          |_<package_content ...>

1. Create **a ROS2 workspace**.

.. code-block:: bash

   cd $HOME
   mkdir -p epd_ros2_ws/src && cd epd_ros2_ws/src

2. Download from **GitHub**.

.. code-block:: bash

   cd epd_ros2_ws/src
   git clone https://github.com/ros-industrial/easy_perception_deployment

3. Install **ONNX Runtime**. Please run the command based on your decision on whether you
intend to run it with CPU or GPU.

.. code-block:: bash

   cd ~/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment/scripts
   sudo chmod u+x install_dep_cpu.bash
   sudo chmod u+x install_dep_gpu.bash

.. code-block:: bash

   # If you intend to only use CPU (better compatibility but slower performance.)
   ./install_deb_cpu.bash
   # OR If you intend to use GPU (lower compatibility but faster performance.)
   ./install_deb_gpu.bash

4. Generate **desktop shortcut**. A ``.desktop`` file should appear in your Desktop.

.. code-block:: bash

   cd ~/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment/
   sudo chmod u+x scripts/create_desktop_shortcut.bash
   ./scripts/create_desktop_shortcut.bash
