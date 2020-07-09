# Autoware in Carla

The carla autoware bridge is now hosted and maintained [here](https://gitlab.com/autowarefoundation/autoware.ai/simulation/-/tree/master/carla_simulator_bridge).

This repository contains additional files (e.g. launch files).

# Carla-Autoware Docker image
Build a Docker image which integrates the Carla Autoware Bridge into the Autoware Docker image

## Requirements

- Docker 
- Nvidia container runtime:
Installation: https://github.com/NVIDIA/nvidia-container-runtime#installation
Configuration: https://github.com/NVIDIA/nvidia-container-runtime#docker-engine-setup
- NVidia Docker V2: https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)

### Build image
Clone the Carla Autoware Bridge GIT repository and initialize the GIT submodules.

    #ensure that the GIT repo is clean
    git clean -fdx

    cd docker
    ./build.sh <optional "docker build ..." parameters, e.g. setting build-args like proxies>

These commands will create a "carla-autoware:latest" Docker image.

### Run image

1. Run carla in a docker

Please consult the main README regarding execution of Carla, the Carla Autoware Bridge and the
Autoware stack.
Start Carla server as described there.
 
The bridge and the Autoware stack have to be executed within the Docker container. To start it:

    <in docker directory>
    ./run.sh

Within the Docker shell, start Autoware (including the carla-autoware-bridge):

    roslaunch $CARLA_AUTOWARE_ROOT/devel.launch

