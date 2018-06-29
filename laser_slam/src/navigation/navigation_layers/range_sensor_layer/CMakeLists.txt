cmake_minimum_required(VERSION 2.8.3)
project(range_sensor_layer)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  sensor_msgs
  pluginlib)


file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})


add_message_files(
   FILES
   sonar_msgs.msg
#   Message2.msg
 )
 generate_messages(
   DEPENDENCIES
   #geometry_msgs#   move_base_msgs#
   std_msgs sensor_msgs
 )


generate_dynamic_reconfigure_options(cfg/RangeSensorLayer.cfg)

catkin_package(
CATKIN_DEPENDS sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add range sensor layer
add_library(${PROJECT_NAME} src/range_sensor_layer.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS range_sensor_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

#add sonar sensor layer
add_library(sonar_sensor_layer src/sonar_sensor_layer.cpp)
add_dependencies(sonar_sensor_layer ${PROJECT_NAME}_gencfg)
target_link_libraries(sonar_sensor_layer ${catkin_LIBRARIES})

install(TARGETS sonar_sensor_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

#add point laser sensor layer
add_library(point_sensor_layer src/point_sensor_layer.cpp)
add_dependencies(point_sensor_layer ${PROJECT_NAME}_gencfg)
target_link_libraries(point_sensor_layer ${catkin_LIBRARIES})

install(TARGETS point_sensor_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(PROGRAMS scripts/send_message.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
