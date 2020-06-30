cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_agent)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

