cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  tf
  std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
)

add_service_files(FILES
  localization_register.srv
  localization_state.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  INCLUDE_DIRS include
)

add_executable(localization
  include/localization/localization_sub_params.h
  include/localization/localization.h
  src/localization_node.cpp
  src/localization_modules.cpp
  src/localization_params.cpp
  src/localization_json.cpp
  src/localization_sub_params.cpp
)
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization
  ${catkin_LIBRARIES}
  json_linux-gcc-4.8_libmt
  yaml-cpp
)

install(TARGETS localization
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY configs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
)

install(FILES
   launch/localization.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )