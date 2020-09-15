#!/usr/bin/env bash

source /opt/ros/eloquent/setup.bash

while [ "$(ros2 node list | grep processer)" ]
do
  ros2 topic pub /processor/state_input std_msgs/msg/String "data: shutdown" --once
done

while [ "$(ros2 node list | grep image_viewer)" ]
do
  ros2 topic pub /image_viewer/state_input std_msgs/msg/String "data: shutdown" --once
done
