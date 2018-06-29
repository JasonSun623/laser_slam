# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(mrobot_local_planner)

find_package(catkin REQUIRED 
  COMPONENTS
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
  INCLUDE_DIRS include
  LIBRARIES mrobot_local_planner
)
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

# build the library
add_library(mrobot_local_planner 
  src/mrobot_local_planner_ros.cpp 
  src/mrobot_local_planner.cpp 
  src/conversions_and_types.cpp 
  src/mrobot_visualization.cpp 
  src/mrobot_trajectory_controller.cpp
)

target_link_libraries(mrobot_local_planner
  json_linux-gcc-4.8_libmt
  ${catkin_LIBRARIES})

install(
  DIRECTORY include/mrobot_local_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS mrobot_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES elp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
