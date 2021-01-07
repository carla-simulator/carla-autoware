cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_agent)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

install(DIRECTORY agent/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/agent
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

