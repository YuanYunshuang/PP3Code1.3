cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(GridMapRegistration)
#add_definitions(-std=c++11)
find_package(PCL  REQUIRED )
find_package(OpenCV  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(gridMapRegistration src/main.cpp)
target_link_libraries(gridMapRegistration  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
