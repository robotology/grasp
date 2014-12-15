# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(appname GraspModuleApp)
set(contextnames puma icub)

foreach(contextname ${contextnames}) 

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/${contextname}/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${contextname}/*.template)

#if(UNIX)
#	if(NOT (contextname EQUAL puma))
#		message(STATUS "Copying startup conevnience script for linux - iCub")
#		set(scripts ${scripts} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${contextname}/darwin_start.sh)
#	endif()
#endif()

if (ICUB_FOUND)
icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts)
endif()


yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/${contextname})

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

endforeach(contextname)

yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ymanager.ini" DESTINATION "${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/..")
