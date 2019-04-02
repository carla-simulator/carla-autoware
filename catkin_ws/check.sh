#!/bin/bash -e
autopep8 src/carla_autoware_bridge/src/carla_autoware_bridge/* --in-place --max-line-length=100
autopep8 src/carla_autoware_waypoint_publisher/src/carla_autoware_waypoint_publisher/* --in-place --max-line-length=100
autopep8 src/carla_client/src/carla_client/* --in-place --max-line-length=100

FILES="$(ls src/carla_autoware_bridge/src/carla_autoware_bridge/*) $(ls src/carla_autoware_waypoint_publisher/src/carla_autoware_waypoint_publisher/*) $(ls src/carla_client/src/carla_client/*) "

pylint --rcfile=.pylintrc $FILES
