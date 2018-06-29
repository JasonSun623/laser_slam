cmake_minimum_required(VERSION 2.8.3)
project(lane_follower)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules 
	nav_core 
	base_local_planner 
	costmap_2d 
	nav_msgs
	pluginlib
	message_generation
    	roscpp
    	dynamic_reconfigure
    	tf
)

generate_dynamic_reconfigure_options(
  cfg/LaneFollowingPlanner.cfg
)


#add_service_files(
#  FILES
#  missions.srv
#)
#generate_messages(
#  DEPENDENCIES
# std_msgs
#)
catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    base_local_planner
    nav_msgs
    motion_planner
    pluginlib
    roscpp
    tf
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(lane_follower 
  src/lane_follower.cpp
)
target_link_libraries(lane_follower
    yaml-cpp
    json_linux-gcc-4.8_libmt
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_library(lane_following_planner src/lane_following_planner.cpp)
target_link_libraries(lane_following_planner ${catkin_LIBRARIES})

add_executable(lane_follower_node
  src/lane_follower_node.cpp
)

target_link_libraries(lane_follower_node lane_follower )

#add_executable(laser_tf_publisher_node
#  src/laser_tf_publisher.cpp
#)
#target_link_libraries(laser_tf_publisher_node ${catkin_LIBRARIES})
install(TARGETS lane_following_planner lane_follower_node lane_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES lane_following_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch configs
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

