#!/bin/bash
set -e

source /home/autoware/carla-autoware/catkin_ws/devel/setup.bash
/opt/ros/kinetic/bin/roscore &

exec "$@"
