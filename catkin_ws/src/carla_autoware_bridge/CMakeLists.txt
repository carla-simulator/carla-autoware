cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_bridge)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  rospy
)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        rospy
)

catkin_install_python(PROGRAMS
    src/carla_autoware_bridge/carla_to_autoware_vehicle_status
    src/carla_autoware_bridge/odometry_to_posestamped
    src/carla_autoware_bridge/vehiclecmd_to_ackermanndrive
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

