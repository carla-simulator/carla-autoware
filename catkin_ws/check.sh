#!/bin/bash -e
autopep8 src/carla_autoware_bridge/src/carla_autoware_bridge/* --in-place --max-line-length=100
autopep8 src/carla_autoware_ego_vehicle/src/carla_autoware_ego_vehicle/* --in-place --max-line-length=100

FILES="$(ls src/carla_autoware_bridge/src/carla_autoware_bridge/*) $(ls src/carla_autoware_ego_vehicle/src/carla_autoware_ego_vehicle/*) "

pylint --rcfile=.pylintrc $FILES
