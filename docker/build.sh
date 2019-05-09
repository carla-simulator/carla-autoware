#!/bin/sh

docker build --pull -t carla-autoware -f Dockerfile ./.. "$@"
