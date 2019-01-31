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

    #create Carla ego vehicle (BMW Isetta, to not have reflection of the roof)
    export PYTHONPATH=<path-to-carla>/PythonAPI/carla-<version_and_arch>.egg
    ./carla_autoware_control.py --filter vehicle.bmw.isetta

    #Terminal 3

    source ~/carla-autoware/catkin_ws/devel/setup
    #Execute the following to capture single point clouds within the /tmp/pcl_capture directory
    roslaunch pcl_recorder pcl_recorder.launch


When the capture drive is done, you can reduce the overall size of the point cloud.

    #create one point cloud file
    pcl_concatenate_points_pcd /tmp/pcl_capture/*.pcd

    #filter duplicates
    pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd

    #verify the result
    pcl_viewer map.pcd


