#!/bin/bash

source /home/jetson/env/bin/activate

source /opt/ros/melodic/setup.bash
source ~/catkin_workspace/devel/setup.bash

export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages:$PYTHONPATH
export PYTHONPATH=$HOME/cv_bridge_ws/devel/lib/python3/dist-packages:$PYTHONPATH

python $(rospack find combined_system)/scripts/ros_road_follow.py

deactivate