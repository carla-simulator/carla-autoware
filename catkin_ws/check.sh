#!/bin/bash -e
autopep8 src/carla_autoware_bridge/src/carla_autoware_bridge/* --in-place --max-line-length=100
autopep8 src/carla_autoware_ego_vehicle/src/carla_autoware_ego_vehicle/* --in-place --max-line-length=100
autopep8 src/carla_points_map_loader/src/carla_points_map_loader/* --in-place --max-line-length=100

FILES="$(ls src/carla_autoware_bridge/src/carla_autoware_bridge/*) $(ls src/carla_autoware_ego_vehicle/src/carla_autoware_ego_vehicle/*)  $(ls src/carla_points_map_loader/src/carla_points_map_loader/*)"

pylint --rcfile=.pylintrc $FILES
