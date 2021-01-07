#!/bin/bash

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

docker run \
    -it --rm \
    --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:ro \
    --volume=$(pwd)/carla_autoware_manager:/home/autoware/carla_ws/src/carla-autoware-manager:rw \
    --volume=$(pwd)/carla_autoware_agent:/home/autoware/carla_ws/src/carla_autoware_agent:rw \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --net=host \
    $RUNTIME \
    carla-autoware:latest

