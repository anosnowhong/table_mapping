cmake_minimum_required(VERSION 2.8)
project(table_mapping)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/primitive_extraction)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ransac_primitives)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/control_morse)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/table_detection)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/table_registration)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/PCLV)



