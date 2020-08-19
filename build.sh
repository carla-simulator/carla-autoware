#!/bin/sh
CARLA_VERSION=CARLA_0.9.10-Pre_Ubuntu18
CARLA_RELEASE_URL=https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/${CARLA_VERSION}.tar.gz

mkdir carla-simulator && cd carla-simulator
curl "${CARLA_RELEASE_URL}" | tar xz

cd ..
docker build -t carla-autoware -f Dockerfile . "$@"

