project(pclvCmake)
cmake_minimum_required(VERSION 2.8)

find_package(Qt4 REQUIRED)
include (${QT_USE_FILE})

find_package(eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${QT_DEFINITIONS})

set(test_HEADERS openni_grabber1.h  widget.h  normal_pointrepresentation.h gsurface.h registration1.h
	construction.h
	)
set(test_UIS widget.ui gsurface.ui)
set(test_SRCS main.cpp openni_grabber1.cpp widget.cpp  normal_pointrepresentation.cpp gsurface.cpp 
	registration1.cpp
	construction.cpp
	${test_QRCS})

QT4_WRAP_CPP(test_HEADERS_MOC ${test_HEADERS})
QT4_WRAP_UI(test_UIS_UIC ${test_UIS})
QT4_ADD_RESOURCES(test_QRC_RCC ${test_QRCS})

add_executable(simple_cloud ${test_SRCS} ${test_HEADERS_MOC} ${test_UIS_UIC} ${test_QRC_RCC}
)
target_link_libraries(simple_cloud ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} QVTK )
