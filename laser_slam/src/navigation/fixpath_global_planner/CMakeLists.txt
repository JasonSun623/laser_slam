cmake_minimum_required(VERSION 2.8.3)
project(fixpath_global_planner)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        map_server
        )

find_package(Eigen REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        fixpath_global_planner
    CATKIN_DEPENDS
        nav_core
        roscpp
        pluginlib
	map_server
	std_msgs
)

add_library (fixpath_global_planner src/fixpath_global_planner.cpp src/fixpath_global_planner_ros.cpp)
target_link_libraries(fixpath_global_planner
    ${catkin_LIBRARIES}
    )
add_dependencies(fixpath_global_planner ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(fixpath_global_planner_ros src/fixpath_global_planner_ros.cpp)
target_link_libraries(fixpath_global_planner_ros
    fixpath_global_planner
)

install(TARGETS fixpath_global_planner fixpath_global_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/fixpath_global_planner/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES fixpath_global_planner.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

### The problem with FindFLTK is that it only reports success if *all*
### fltk components are installed, but we only need the core library.
# include (FindFLTK)
#include (CheckIncludeFileCXX)
#check_include_file_cxx (FL/Fl.H NAVFN_HAVE_FLTK)
#check_include_file_cxx (pgm.h NAVFN_HAVE_NETPBM)
message (STATUS "NAVFN_HAVE_FLTK: ${NAVFN_HAVE_FLTK}, NETPBM: ${NAVFN_HAVE_NETPBM}")
