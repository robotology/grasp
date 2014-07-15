# Copyright: 2010 RobotCub Consortium
# Author: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.8)

project(grasp)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(PCL 1.6 REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

add_definitions(${YARP_DEFINES}) 

include(YarpInstallationHelpers)
find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_subdirectory(lib)

add_subdirectory(object-reconstruction)

add_subdirectory(power-grasp)

add_subdirectory(precision-grasp)

icubcontrib_finalize_export(grasp)
icubcontrib_add_uninstall_target()