cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_waypoint_publisher)

catkin_package()


install(PROGRAMS
  src/odometry_to_posestamped
  src/vehiclecmd_to_ackermanndrive
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
