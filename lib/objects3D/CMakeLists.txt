set(PROJECTNAME objects3D)
project(${PROJECTNAME})

INCLUDE(YarpIDL)
SET(generated_files_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(iCub/data3D/objects3D.thrift ${generated_files_dir})

set(THRIFT_HEADERS)
set(THRIFT_SOURCES)

set(fileList iCub_data3D_objects3D_thrift.cmake)
foreach(scriptFile ${fileList})
  include(${scriptFile})
  list(APPEND THRIFT_HEADERS ${headers})
  list(APPEND THRIFT_SOURCES ${sources})
endforeach()

include_directories(include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${THRIFT_HEADERS} ${THRIFT_SOURCES})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)

icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
										  EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
										  DESTINATION include/iCub/data3D
										  FILES ${THRIFT_HEADERS})

