cmake_minimum_required(VERSION 2.8.3)
project(base_cmdvel_mux)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS include
)

add_executable(base_cmdvel_mux
  include/base_cmdvel_mux/base_cmdvel_mux.h
  src/base_cmdvel_mux.cpp
)
add_dependencies(base_cmdvel_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_cmdvel_mux
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS base_cmdvel_mux
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY configs/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
