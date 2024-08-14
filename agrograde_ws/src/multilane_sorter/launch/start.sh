#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/agrograde_ws/devel/setup.bash
export ROS_ENV_LOADER=/etc/ros/env.sh
source ~/.bashrc
nohup roslaunch multilane_sorter onion.launch
