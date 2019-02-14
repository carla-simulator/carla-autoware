cmake_minimum_required(VERSION 2.8.3)
project(carla_client)

catkin_package()

install(PROGRAMS
  src/carla_autoware_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
