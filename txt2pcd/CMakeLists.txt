cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_CXX_FLAGS "-std=c++11")

project(txt2pcd)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

file(GLOB native_srcs "src/*.cpp")

add_executable(${PROJECT_NAME} ${native_srcs})


target_link_libraries(${PROJECT_NAME}
   ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
 )


