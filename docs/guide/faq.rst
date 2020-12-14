.. _faq:

FAQ
====
This article lists answers to frequently asked questions.

Issue: How to kill the program faster?
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
The **easy_perception_deployment** ROS2 package may take a long time to close when issued a kill signal via the Graphical User Interface (GUI).

To kill it faster, do the following:

Click on the terminal running the nodes.
Press Ctrl + c to kill all nodes.

Issue: How to view easy_perception_deployment's custom messages?
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
The **easy_perception_deployment** ROS2 package outputs streamlined custom messages that can be used by external robotic agents in the following ROS2 topics:

.. code-block:: bash

    /processor/epd_p1_output
    /processor/epd_p2_output
    /processor/epd_p3_output

Follow the instructions below to view these messages:

.. code-block:: bash

    # Open a new terminal
    source /opt/ros/foxy/setup.bash
    source ./epd_msgs/install/setup.bash
    # For Precision Level 1 output
    ros2 topic echo /processor/epd_p1_output
    # For Precision Level 2 output
    ros2 topic echo /processor/epd_p2_output
    # For Precision Level 3 output
    ros2 topic echo /processor/epd_p3_output
