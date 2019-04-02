cmake_minimum_required(VERSION 2.8.3)
project(rgbd_slam)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  kinect2_bridge
  message_filters
  roscpp
  rostime
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(G2O REQUIRED)


add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rgbd_slam
  CATKIN_DEPENDS cv_bridge image_transport kinect2_bridge message_filters roscpp rostime sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/ORB_SLAM2_modified
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node src/rgbd_slam_node.cpp)
add_executable(saveTUMDataset src/saveTUMDataset.cpp)
add_executable(rgbd_tum src/rgbd_tum.cpp)
add_executable(ctm_slam src/ctm_slam.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}  
   ${EIGEN3_LIBS} 
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/ORB_SLAM2_modified/lib/libORB_SLAM2.so
   ${PROJECT_SOURCE_DIR}/ORB_SLAM2_modified/Thirdparty/DBoW2/lib/libDBoW2.so
   g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
 )

 target_link_libraries(rgbd_tum
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}  
   ${EIGEN3_LIBS} 
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/ORB_SLAM2_modified/lib/libORB_SLAM2.so
   ${PROJECT_SOURCE_DIR}/ORB_SLAM2_modified/Thirdparty/DBoW2/lib/libDBoW2.so
   g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
 )


target_link_libraries(saveTUMDataset ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES})

target_link_libraries(ctm_slam ${catkin_LIBRARIES}
                               ${OpenCV_LIBRARIES}
                               ${PCL_LIBRARIES}
                               ${EIGEN3_LIBS} )

