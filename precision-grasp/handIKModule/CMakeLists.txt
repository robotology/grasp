set(PROJECTNAME handIKModule)
project(${PROJECTNAME})

find_package(IPOPT REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${IPOPT_INCLUDE_DIRS})

set(folder_source src/handIK.cpp src/handIKModule.cpp src/main.cpp src/testHandIK.cpp)
set(folder_header include/handIK.h include/handIKModule.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_definitions(${IPOPT_DEFINITIONS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
set_property(TARGET ${PROJECTNAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} iKin ${IPOPT_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

