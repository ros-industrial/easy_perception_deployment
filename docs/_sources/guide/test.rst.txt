.. _test:

Test
====
This article is intended for future developers.
The instructions here will guide you on how to run the various automated test suites on **easy_perception_deployment**.

Run *gtests* on Easy Perception Deployment ROS2 package
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
The **easy_perception_deployment** ROS2 package contains both C++ and python source codes.
However, only the **ROS2 inference engine implementation is in C++**.

Run the following commands:

.. code-block:: bash

   cd ~/epd_ros2_ws
   colcon build
   colcon test --event-handlers console_direct+

Generate *gtests* Code Coverage
+++++++++++++++++++++++++++++++
Run the following commands:

.. code-block:: bash

   colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'
   colcon lcov-result --initial
   colcon test --packages-select easy_perception_deployment --packages-skip epd_msgs
   colcon lcov-result --packages-skip epd_msgs

View the report generated.

.. code-block:: bash

   firefox /lcov/index.html


Run *pytests* on Easy Perception Deployment GUI
+++++++++++++++++++++++++++++++++++++++++++++++++++++
The **easy_perception_deployment** ROS2 package contains both C++ and python source codes.
However, only the **GUI implementation is in Python**.

Run the following commands:

.. code-block:: bash

   cd ~/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment/gui
   sudo chmod u+x run_test_gui.bash
   ./run_test_gui.bash

By running the bash script above, it will automatically **run pytests** and **generate the code coverage report**.

Generate *pytests* Code Coverage
++++++++++++++++++++++++++++++++
Run the same command above.

.. code-block:: bash

   cd ~/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment/gui
   sudo chmod u+x run_test_gui.bash
   ./run_test_gui.bash

The code coverage report should be printed on the same commandline terminal you
have ran the commands on.
