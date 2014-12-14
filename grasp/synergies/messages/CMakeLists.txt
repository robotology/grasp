# Copyright (C) 2013 King's College London
# Authors: Kris De Meyer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Include this file into your own CMake project files to use DarwinMessages

cmake_minimum_required(VERSION 2.6)

# Make sure we can call this file by itself or include it from another place
if(NOT DEFINED MESSAGES)
	set(MESSAGES "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# add the current folder as an include directory
include_directories("${MESSAGES}")

# add the required sub directories
add_subdirectory("${MESSAGES}/TypeSafeBottle" "${CMAKE_CURRENT_BINARY_DIR}/TypeSafeBottle")
add_subdirectory("${MESSAGES}/MessageFormats" "${CMAKE_CURRENT_BINARY_DIR}/MessageFormats")
