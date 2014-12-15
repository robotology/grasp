# Copyright (C) 2013 King's College London
# Authors: Kris De Meyer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

# add a target that will not generate a file
add_custom_target(TypeSafeBottle 
	SOURCES TypeSafeBottle.h YarpTypeSafeBottle.xsd YarpTypeSafeBottle.xslt)

#add files to the correct folders in VS
source_group("Header Files" FILES TypeSafeBottle.h)
source_group("Source Files" FILES YarpTypeSafeBottle.xsd YarpTypeSafeBottle.xslt)




