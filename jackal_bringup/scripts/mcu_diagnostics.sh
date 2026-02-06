#!/bin/bash
export ROS_DOMAIN_ID=7
cd ${HOME}/mobi_robots
source install/setup.bash

ros2 launch jackal_description jackal_diagnostic.launch.py namespace:=j100_0929
