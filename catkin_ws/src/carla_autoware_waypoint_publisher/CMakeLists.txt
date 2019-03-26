cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_waypoint_publisher)

find_package(catkin REQUIRED COMPONENTS
        rospy
)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        rospy
)

catkin_install_python(PROGRAMS
    src/carla_autoware_waypoint_publisher/carla_autoware_waypoint_publisher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
