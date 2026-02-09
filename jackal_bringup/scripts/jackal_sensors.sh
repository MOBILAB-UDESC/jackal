#!/bin/bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd ${HOME}/mobi_robots
source install/setup.bash

ros2 launch jackal_bringup jackal_sensors.launch.py
