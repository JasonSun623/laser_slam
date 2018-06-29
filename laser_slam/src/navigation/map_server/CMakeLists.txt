cmake_minimum_required(VERSION 2.8.3)
project(map_server)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            ar_track_alvar
            tf
            nav_msgs
            convert
            gmapping
        )

find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

set(CMAKE_CXX_FLAGS "-std=c++11")

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})


add_message_files(
    DIRECTORY msg
    FILES
    point.msg
)

add_service_files(
   FILES
   query_uwb.srv
   query_all_points.srv
   GetFixPath.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
   geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        image_loader
	map_list_manager
    CATKIN_DEPENDS
        ar_track_alvar
        roscpp
        tf
        nav_msgs
        convert
)

include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image ${Boost_LIBRARIES})

add_library(map_list_manager src/map_list_manager.cpp)

add_library(regions
	src/region.cpp
	src/database_manager.cpp
	src/region_manager.cpp
)
target_link_libraries(regions
    json_linux-gcc-4.8_libmt
    sqlite3
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(cmd_processor src/cmd_processor.cpp)

add_executable(map_server_mrobot src/main_mrobot.cpp)
target_link_libraries(map_server_mrobot
    regions
    map_list_manager
    image_loader
    cmd_processor
    yaml-cpp
    json_linux-gcc-4.8_libmt
    ${catkin_LIBRARIES}
)


# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()



## Tests
if(CATKIN_ENABLE_TESTING)
  copy_test_data( FILES
      test/testmap.bmp
      test/testmap.png )
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp test/test_constants.cpp)
  target_link_libraries(${PROJECT_NAME}_utest image_loader SDL SDL_image)

  add_executable(rtest test/rtest.cpp test/test_constants.cpp)
  target_link_libraries( rtest
      gtest
      ${catkin_LIBRARIES}
  )
  add_dependencies(rtest nav_msgs_gencpp)

  # This has to be done after we've already built targets, or catkin variables get borked
  find_package(rostest)
  add_rostest(test/rtest.xml)
endif()

## Install executables and/or libraries
install(TARGETS image_loader map_server_mrobot map_list_manager cmd_processor regions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY maps/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
## Install excutable python script
install( 
    PROGRAMS
      scripts/crop_map
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
