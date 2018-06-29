cmake_minimum_required(VERSION 2.8.3)
project(move_base)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        move_base_msgs
        nav_core
		fixpath_global_planner
        tf
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

add_service_files(
   FILES
   Pause.srv
   ConfigParam.srv
   FixpathNav.srv
)

generate_messages(
   DEPENDENCIES
   #sensor_msgs#   std_msgs
   #robot_state
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
		fixpath_global_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# move_base
add_library(move_base
  src/move_base.cpp
)
target_link_libraries(move_base
    yaml-cpp
    json_linux-gcc-4.8_libmt
	fixpath_global_planner
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base move_base_gencfg)
add_dependencies(move_base geometry_msgs_gencpp)

add_executable(move_base_node
  src/move_base_node.cpp
)
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

install(DIRECTORY launch configs
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    PROGRAMS
       scripts/subtopic_forwarder.py
       scripts/subtopic_forwarder_node.py
       scripts/warner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base
        move_base_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
