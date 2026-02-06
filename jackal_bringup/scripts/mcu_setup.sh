#!/bin/bash
cd ${HOME}/mobi_robots
source install/setup.bash

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch jackal_description jackal_mcu_setup.launch.py namespace:=j100_0929

