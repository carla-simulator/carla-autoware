# Autoware in CARLA

The carla autoware bridge is now hosted and maintained [here](https://github.com/Autoware-AI/simulation/tree/master/carla_simulator_bridge).

This repository contains a demonstrator of an autoware agent ready to be executed with CARLA.

## CARLA autoware agent
The autoware agent is provided as a ROS package. All the configuration can be found inside the `carla-autoware-agent` folder.

The easiest way to run the agent is by building and running the provided docker image.

### Requirements

- Docker (19.03+)
- Nvidia docker. (https://github.com/NVIDIA/nvidia-docker)

### Setup

Firstly clone the carla autoware repository alongside the needed [autoware contents](https://bitbucket.org/carla-simulator/autoware-contents.git):

```sh
git clone --recurse-submodules https://github.com/carla-simulator/carla-autoware
```

Afterwards, build the image with the following command:

```sh
./build.sh
```

This will generate a `carla-autoware:latest` docker image.

### Run the agent

1. Run CARLA in a docker container.

To start a CARLA server within a docker container run the following command:

```sh
docker run -p 2000-2002:2000-2002 --runtime=nvidia --gpus all carlasim/carla:0.9.9
```

You may find more information about running CARLA using docker [here](https://carla.readthedocs.io/en/latest/build_docker/)

2. Run the `carla-autoware` image: 

```sh
./run.sh
```

This will start an interactive shell inside the container. To start the agent run the following command:

```sh
roslaunch carla_autoware_agent carla_autoware_agent.launch
```
