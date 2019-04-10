cmake_minimum_required(VERSION 2.8.3)
project(carla_points_map_loader)

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
  src/carla_points_map_loader/carla_points_map_loader
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
