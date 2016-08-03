cmake_minimum_required(VERSION 2.8)
project(pcl_tools)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

set(SOURCE_FILES
    src/remove_nan.cpp)

add_executable(nan_remove ${SOURCE_FILES})
target_link_libraries(nan_remove ${PCL_LIBRARIES})