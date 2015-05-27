# Copyright: 2010 RobotCub Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME precision-grasp)

project(${PROJECTNAME})

add_subdirectory(app)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${objects3D_INCLUDE_DIRS})
include_directories(${minimumBoundingBox_INCLUDE_DIRS})
include_directories(${forceClosure_INCLUDE_DIRS})

set(header include/precisionGrasp.h include/visualizationThread.h include/psoThread.h)

if(NOT UNIX)
	set(header ${header}
		include/custom/dirent.h)
endif()
file(GLOB source src/*.cpp)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECTNAME} ${source} ${header})
target_link_libraries (${PROJECTNAME} ${PCL_LIBRARIES} ${YARP_LIBRARIES} minimumBoundingBox ctrlLib forceClosure objects3D iKin)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

