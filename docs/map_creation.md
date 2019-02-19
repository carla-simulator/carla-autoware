# Map Creation

Autoware uses a pointcloud map. You can easily create it by manually driving around in Carla.

## Setup

See setup of carla-autoware.


## Run

Execute the Carla Simulator and an Ego Vehicle.

    #Terminal 1

    #execute Carla
    SDL_VIDEODRIVER=offscreen <path-to-carla>/CarlaUE4.sh /Game/Carla/Maps/Town01 -benchmark -fps=10

    #Terminal 2

    #setup a Carla client and the pcl capturing.
    #The captured point clouds are saved within /tmp/pcl_capture directory.
    export PYTHONPATH=<path-to-carla>/PythonAPI/carla-<version_and_arch>.egg:<path-to-carla>/PythonAPI/
    source $CARLA_AUTOWARE_ROOT/catkin_ws/devel/setup.bash
    roslaunch carla_autoware_bridge carla_autoware_bridge_capture_pcl_map.launch


When the capture drive is done, you can reduce the overall size of the point cloud.

    #create one point cloud file
    pcl_concatenate_points_pcd /tmp/pcl_capture/*.pcd

    #filter duplicates
    pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd

    #verify the result
    pcl_viewer map.pcd


