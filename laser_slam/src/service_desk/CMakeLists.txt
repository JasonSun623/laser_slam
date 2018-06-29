cmake_minimum_required(VERSION 2.8.3)
project(service_desk)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package(

)

install(PROGRAMS
  scripts/service_desk.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

