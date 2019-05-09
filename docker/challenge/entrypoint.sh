#!/bin/bash
set -e
echo "carla-autoware-challenge: ENTRYPOINT"
echo "--- sourcing setup script"
source /home/autoware/carla-autoware/catkin_ws/devel/setup.bash
echo "--- launching roscore"
/opt/ros/kinetic/bin/roscore &
echo "--- executing CMD"
exec "$@"
