cmake_minimum_required(VERSION 2.8.3)
project(table_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  mongodb_store
)

find_package(Eigen3)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

include_directories(include)

add_message_files(
   FILES
   table_neighbour.msg
   table_neighbour_arr.msg
   Table.msg
   table_merge_info.msg
 )

add_service_files(
  FILES
  db_extract.srv
  db_extract_whole_table.srv
  db_table_clouds.srv
  db_table.srv
  db_merge.srv
  db_table_centre.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES table_core
  CATKIN_DEPENDS
  DEPENDS OpenCV PCL
)

file(GLOB SRC_FILES include/${PROJECT_NAME}/*.cpp)


add_library(table_core ${SRC_FILES})

target_link_libraries(table_core
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )


add_executable(db_cloud_extraction src/db_cloud_extraction.cpp)
add_dependencies(db_cloud_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(table_status src/table_status.cpp)
add_dependencies(table_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(table_all src/table_all.cpp)
add_dependencies(table_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(db_table_extraction src/db_table_extraction.cpp)
#add_dependencies(db_table_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(db_table_maintaining src/db_table_maintaining.cpp)
#add_dependencies(db_table_maintaining ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(viz_convex src/viz_convex.cpp)
add_dependencies(viz_convex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(data_change test/neighbour_change.cpp)
add_dependencies(data_change ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(centreprint test/centreprint.cpp)
add_dependencies(centreprint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(convexsize test/convex_size.cpp)
add_dependencies(convexsize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(db_cloud_extraction
        table_core
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

target_link_libraries(table_status
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

target_link_libraries(table_all
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

#target_link_libraries(db_table_extraction
#        table_core
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${OpenCV_LIBS}
#        )

#target_link_libraries(db_table_maintaining
#        table_core
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${OpenCV_LIBS}
#        )

target_link_libraries(viz_convex
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

target_link_libraries(data_change
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
target_link_libraries(centreprint
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
target_link_libraries(convexsize
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
