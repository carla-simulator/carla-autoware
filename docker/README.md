# Carla-Autoware Docker image
Build a Docker image which integrates the Carla Autoware Bridge into the Autoware Docker image

## Requirements

- Docker
- NVidia Docker V2: https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)


### Build image
Clone the Carla Autoware Bridge GIT repository and initialize the GIT submodules.

    #ensure that the GIT repo is clean
    git clean -fdx

    cd docker
    ./build.sh <optional "docker build ..." parameters, e.g. setting build-args like proxies>

These commands will create a "carla-autoware:latest" Docker image.


### Run image
Please consult the main README regarding execution of Carla, the Carla Autoware Bridge and the
Autoware stack.
Start Carla server as described there (Terminal 1 and 2).

The bridge and the Autoware stack have to be executed within the Docker container (the
Terminal 3 steps).

Start the container:

    <in docker directory>
    ./run.sh

Within the Docker shell:

    roslaunch $CARLA_AUTOWARE_ROOT/devel.launch

Then follow the remaining steps from main README, especially the ones within Autoware Runtime Manager.

