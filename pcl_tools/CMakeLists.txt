cmake_minimum_required(VERSION 2.8)
project(pcl_tools)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)


set(CMAKE_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(nan_remove ${CMAKE_CURRENT_SOURCE_DIR}/remove_nan.cpp)
target_link_libraries(nan_remove ${PCL_LIBRARIES})

add_executable(force_dense ${CMAKE_CURRENT_SOURCE_DIR}/force_dense.cpp)
target_link_libraries(force_dense ${PCL_LIBRARIES})

add_executable(outliers ${CMAKE_CURRENT_SOURCE_DIR}/remove_outliers.cpp)
target_link_libraries(outliers ${PCL_LIBRARIES})
