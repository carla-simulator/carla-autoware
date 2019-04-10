# Autoware in Carla
Integration of AutoWare AV software with the CARLA simulator

## Requirements

- ROS kinetic
- Autoware (tested with 1.11.0)

## Opens

- object detection (especially traffic lights)
- no compliance with traffic rules (due to missing vector map)

## Setup

### Autoware

Setup/build Autoware as described here: https://github.com/CPFL/Autoware

### Download Point cloud maps

    # Download pointcloud maps for Carla Towns
    cd ~
    git clone https://bitbucket.org/carla-simulator/autoware-contents

### Carla

    #download docker image (e.g. version 0.9.5)
    docker pull carlasim/carla:<carla-version>

    #extract the Carla Python API from the image
    cd ~
    mkdir carla-python
    docker run --rm --entrypoint tar carlasim/carla:<carla-version> cC /home/carla/PythonAPI . | tar xvC ~/carla-python


### Carla Autoware Bridge

The Carla Autoware Bridge is a ROS package. Therefore we create a catkin workspace (containing all relevant packages).
The generic Carla ROS bridge (https://github.com/carla-simulator/ros-bridge.git) is included as GIT submodule and 
has to be initialized ("git submodule update --init") and updated ("git submodule update").

    cd ~
    git lfs clone https://github.com/carla-simulator/carla-autoware.git
    cd carla-autoware
    git submodule update --init
    cd catkin_ws
    catkin_init_workspace src/
    cd src
    ln -s <path-to-autoware>/ros/src/msgs/autoware_msgs
    cd ..
    catkin_make

## Run

To run Autoware within Carla please use the following execution order:

1. Carla Server
2. Autoware Runtime Manager
3. Carla Autoware Bridge
4. Autoware Stack

You need two terminals:

    #Terminal 1

    #execute Carla
    #For details, please refer to the CARLA documentation
    nvidia-docker run -p 2000-2001:2000-2001 -it --rm carlasim/carla:<carla-version> ./CarlaUE4.sh /Game/Carla/Maps/Town01 -benchmark -carla-server -fps=20


    #Terminal 2

    export CARLA_AUTOWARE_ROOT=~/carla-autoware
    export CARLA_MAPS_PATH=~/autoware-contents/maps
    source <path-to-autoware>/ros/install/setup.bash
    source $CARLA_AUTOWARE_ROOT/catkin_ws/devel/setup.bash
    export PYTHONPATH=$PYTHONPATH:~/carla-python/carla/dist/carla-0.9.5-py2.7-linux-x86_64.egg:~/carla-python/carla/
    
    #execute Autoware (forks into background)
    <path-to-autoware>/ros/run

    #execute carla-autoware-bridge and carla-autoware-bridge
    #Option1: execute a headless Carla client
    roslaunch carla_autoware_bridge carla_autoware_bridge.launch
    #Option2: including visualization of the ego vehicle and the ability to drive 
    roslaunch carla_autoware_bridge carla_autoware_bridge_with_manual_control.launch

In Autoware Runtime Manager, select the customized launch files:

![Autoware Runtime Manager Settings](docs/images/autoware-runtime-manager-settings.png)

In Autoware Runtime Manager, start rviz and open the configuration <autoware-dir>/ros/src/.config/rviz/default.rviz

Now you can start the Autoware Stack by starting all launch files from top to bottom. The car should start moving.

![Autoware Runtime Manager Settings](docs/images/autoware-rviz-carla-town01-running.png)

A special camera is positioned behind the car to see the car and its environment.
You can subscribe to it via ```/carla/ego_vehicle/camera/rgb/viewFromBehind/image_color```.

### Multi machine setup

You can run Autoware and Carla on different machines. 
To let the carla autoware bridge connect to a remote Carla Server, execute roslaunch with the following parameters

    roslaunch host:=<hostname> port:=<port number> carla_autoware_bridge carla_autoware_bridge.launch


## Development support

### Carla Autoware Ego Vehicle

When starting the carla_autoware_bridge a random spawn point and a fixed goal is used to calculate the route.

To override this, you can use RVIZ.

![Autoware Runtime Manager Settings](docs/images/rviz_set_start_goal.png)

- selecting a Pose with '2D Pose Estimate' will delete the current ego_vehicle and respawn it at the specified position.
- selecting a Pose with '2D Nav Goal' will set a new goal within `carla_waypoint_publisher`.

#### Manual steering

Press `B` to be able to steer the ego vehicle within ROS manual control.

Internally, this is done by stopping the conversion from the Autoware control message to AckermannDrive within the node `vehiclecmd_to_ackermanndrive`. The relevant ros-topic is `/vehicle_control_manual_override`.

## Design

The bridge contains three Carla Clients.

1. ROS Bridge - Monitors existing actors in Carla, publishes changes on ROS Topics (e.g. new sensor data)
2. Ego Vehicle - Instantiation of the ego vehicle with its sensor setup.
3. Waypoint Calculation - Uses the Carla Python API to calculate a route.

![Design Overview](docs/images/design.png)

