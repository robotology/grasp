# Copyright (C) 2013 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD "GraspModule")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 2.6)


set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
    ${ICUB_INCLUDE_DIRS}	
    ${ICUB_CONTRIB_DIRS}
)


# Search for source code.
SET(folder_source 
   src/main.cpp 
   src/GraspModule.cpp
   src/ThreadFactory.cpp
   src/GraspThreadImpl.cpp
   src/IcubThread.cpp
   src/IcubVisThread.cpp
   src/ConfigMessages.xml)
SET(folder_header 
   include/Util.h
   include/Vocabs.h
   include/PrintFunctions.h
   include/PortMap.h
   include/GraspModule.h
   include/ThreadFactory.h
   include/GraspModuleThread.h
   include/GraspThreadImpl.h
   include/IcubThread.h
   include/IcubVisThread.h
   include/ConfigMessages.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    #Set your module target to be dependent on MessageFormats so that it builds later
    #You must put this command AFTER your own add_executable or add_library command
    add_dependencies(${KEYWORD} MessageFormats)

    TARGET_LINK_LIBRARIES(${KEYWORD}        

      ${YARP_LIBRARIES}
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
