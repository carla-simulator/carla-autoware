cmake_minimum_required(VERSION 2.8.3)
project(carla_autoware_manager)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  message_generation
  std_msgs
)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES 
  StartBridge.srv
  StopBridge.srv
  StartAgent.srv
  StopAgent.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# TODO(joel): Add catkin depends?
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/carla_autoware_manager/carla_autoware_manager.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
