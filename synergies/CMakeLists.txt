#Copyright (C) 2013 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea, Kris De Meyer, Giuseppe Cotugno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project (GraspModule)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

find_package(YARP REQUIRED)
find_package(ICUBcontrib QUIET)

# functionality added the 6/11/2013 to cope with previous yarp version
if (${YARP_VERSION} VERSION_LESS 2.3.60)
  set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
endif()

set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${YARP_MODULE_PATH} 
  ${ICUB_MODULE_PATH}
  ${ICUBCONTRIB_MODULE_PATH} # note 10/10/13 aligned to the standard Rea
)

if (ICUBcontrib_FOUND)
  include(ICUBcontribHelpers)
  set_contrib_default_prefix() 
else()
  set(ICUBCONTRIB_CONTEXTS_INSTALL_DIR share/darwin/contexts)
  set(ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR share/darwin/templates/applications)
endif() #note 5/11/13 introduced manually to help DARWIN partners


include(YarpInstallationHelpers)
INCLUDE_DIRECTORIES(
    ${YARP_INCLUDE_DIRS} 	
)

find_package(GSL QUIET)
find_package(IPOPT QUIET)

#if(ICUB_FOUND)
#  INCLUDE(iCubOptions)
#  include(iCubHelpers)
#endif()
#qicub_install_with_rpath()

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")


find_path(MESSAGES MessageFormats/GraspMessages.xml 
	HINTS "${CMAKE_CURRENT_SOURCE_DIR}/messages")
MESSAGE(STATUS "Puppa")
MESSAGE(STATUS ${MESSAGES})
include("${MESSAGES}/CMakeLists.txt" NO_POLICY_SCOPE)

#add_subdirectory(TestMessages)


add_subdirectory(app)
add_subdirectory(src)

install(FILES DARWIN_ROOT.ini DESTINATION share/darwin)
