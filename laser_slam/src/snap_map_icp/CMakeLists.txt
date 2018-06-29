cmake_minimum_required(VERSION 2.8.3)
project(snap_map_icp)

find_package(catkin REQUIRED std_msgs nav_msgs laser_geometry sensor_msgs geometry_msgs tf pcl_conversions localization)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


catkin_package(
  CATKIN_DEPENDS
  message_runtime std_msgs nav_msgs laser_geometry sensor_msgs geometry_msgs tf pcl_conversions 
  )

include_directories(${catkin_INCLUDE_DIRS}
  )

add_executable(SnapMapICP src/SnapMapICP.cpp)
add_executable(TFpub src/TFpub.cpp)
add_executable(FreezeCloud2 src/FreezeCloud2.cpp)

target_link_libraries(SnapMapICP ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(TFpub ${catkin_LIBRARIES})
target_link_libraries(FreezeCloud2 ${catkin_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  message_generation
)


 install(TARGETS SnapMapICP
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
