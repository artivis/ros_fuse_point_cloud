cmake_minimum_required(VERSION 2.8.3)
project(ros_fuse_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  ros_img_sync
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

## Declare a cpp library
#add_library(${PROJECT_NAME}
#
#)
#target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#)

## Declare a cpp executable
add_executable(fuse_xyz src/fuse_xyz_node.cpp)
target_link_libraries(fuse_xyz
   ${catkin_LIBRARIES}
)

add_executable(fuse_xyzrgb src/fuse_xyzrgb_node.cpp)
target_link_libraries(fuse_xyzrgb
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_fuse_point_cloud ros_fuse_point_cloud_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

