cmake_minimum_required(VERSION 2.8.3)
project(carla_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_package(
        CATKIN_DEPENDS
        rospy
)

catkin_install_python(PROGRAMS
  src/carla_client/carla_autoware_client
  src/carla_client/carla_autoware_manual_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
