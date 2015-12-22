# Copyright: (C) 2011 RobotCub Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(PROJECTNAME minimumBoundingBox)

project(${PROJECTNAME})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${objects3D_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(header_private include/iCub/data3D/private/helpers.h)

file(GLOB header include/iCub/data3D/*.h)
file(GLOB source src/*.cpp)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header} ${header_private})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -D_USE_MATH_DEFINES)
add_library(${PROJECTNAME} ${source} ${header} ${header_private})
target_link_libraries(${PROJECTNAME} ${PCL_LIBRARIES} ${YARP_LIBRARIES} objects3D)

icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
										  EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${objects3D_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
										  DESTINATION include/iCub/data3D
										  FILES ${header})

