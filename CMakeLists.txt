cmake_minimum_required(VERSION 2.8)
project(table_mapping)


#TODO:
# add round record(currently only implemented in table_centre_group)
#
#
#
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/control_morse)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/table_detection)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/table_registration)


