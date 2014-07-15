set(PROJECTNAME forceClosure)

project(${PROJECTNAME})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB header include/iCub/grasp/*.h)
file(GLOB source src/*.cpp)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_library(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ctrlLib ${YARP_LIBRARIES})

icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
										  EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS}
										  DESTINATION include/iCub/grasp
										  FILES ${header})

