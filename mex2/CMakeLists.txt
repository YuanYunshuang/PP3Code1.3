cmake_minimum_required(VERSION 3.0)
project(mex2)
set(Matlab_ROOT_DIR C:/Programme/MATLAB/R2017a)
find_package(Matlab COMPONENTS MX_LIBRARY MEX_COMPILER)
if(Matlab_FOUND)
message('Matlab found')
else(Matlab_FOUND)
message('matlab not found')
endif(Matlab_FOUND)
#find_package(Qt4 REQUIRED)

#set(PCL_DIR D:/Program Files/PCL 1.6.0/cmake)
find_package(PCL 1.6.0 REQUIRED )
include_directories(${Matlab_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )#
link_directories(${Matlab_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} )#
add_definitions(${PCL_DEFINITIONS} -std=c++11 -fPIC)
set(QTDIR C:/Qt/4.5)
matlab_add_mex(
  NAME test2
  SRC test2.cpp
  LINK_TO  ${PCL_COMMON_LIBRARIES} ${PCL_LIBRARIES} ${Matlab_LIBRARIES}
)

#add_library(${CU_FILE} SHARED ${${CU_FILE}} ${CU_FILE_NAME} ${CMAKE_SOURCE_DIR}/Matlabdef.def)
#target_link_libraries(${CU_FILE} ${Matlab_LIBRARIES})
#set_target_properties(${CU_FILE} PROPERTIES PREFIX "" LINKER_LANGUAGE CXX)

# 32-bit or 64-bit mex

#
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

#find_package(Matlab) 
#add_executable(pcd_write_test test2.cpp)
#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )