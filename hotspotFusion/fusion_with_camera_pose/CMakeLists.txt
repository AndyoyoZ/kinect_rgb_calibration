cmake_minimum_required(VERSION 2.8.3)
#SET(OpenCV_DIR "/home/communicationgroup/opencv/opencv-3.4.2/build")
add_definitions(-std=c++11)

project(cloudFusion)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#添加Eigen头文件
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.3.1 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB native_srcs "src/*.cpp")

add_executable(${PROJECT_NAME} ${native_srcs})


target_link_libraries(${PROJECT_NAME}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


