cmake_minimum_required(VERSION 2.8.3)
project(mr_param)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(mr_param_node
  include/mr_param/mr_param_node.h
  src/mr_param_yaml.cpp
  src/mr_param_node.cpp
)
add_dependencies(mr_param_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mr_param_node
  ${catkin_LIBRARIES}
  json_linux-gcc-4.8_libmt
  yaml-cpp
)

install(TARGETS mr_param_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY configs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)