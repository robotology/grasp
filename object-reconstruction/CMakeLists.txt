set(PROJECTNAME object-reconstruction)

project(${PROJECTNAME})

find_package(stereoVision)
find_package(OpenCV)

add_subdirectory(app)

include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${ICUB_INCLUDE_DIRS}) 
include_directories(${YARP_INCLUDE_DIRS}) 
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${stereoVision_INCLUDE_DIRS})
include_directories(${objects3D_INCLUDE_DIRS}) 
include_directories(${minimumBoundingBox_INCLUDE_DIRS})

#file(GLOB source src/*.cpp)
set(source src/main.cpp src/objectReconstr.cpp src/reconstructionRoutine.cpp)
#file(GLOB header include/*.h)
set(header include/objectReconstr.h include/reconstructionRoutine.h)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} stereoVision objects3D minimumBoundingBox icubmod ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
