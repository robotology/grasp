// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  King's College London
  * Author: Kris De Meyer, Giuseppe Cotugno
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
  
/**
 * @file main.cpp
 * @brief main function for the Observer Stub
 */

//std includes
#include <exception>

//yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

//project includes
#include "PrintFunctions.h"
#include "GraspModule.h" 

using namespace darwin::grasp;

int main(int argc,char* argv[]) {

	const char* policy = "DARWIN_ROOT"; //default, overriden by --policy parameter
	const char* context = "GraspModuleApp/icub"; //default, overridden by --context parameter
	const char* config = "config.ini"; //default, overridden by --from parameter

	PrintLine(yarp::os::ConstString("Resource Finder Policy: ") + policy);
	PrintLine(yarp::os::ConstString("Default Context: ") + context);
	PrintLine(yarp::os::ConstString("Default Config File: ") + config);

	PrintLine("Command Line Arguments: ");
	for(int i=1;i<argc;i+=2) {
		PrintLine(yarp::os::ConstString("\t") + argv[i] + " " + argv[i+1]);
	}

	try {

		PrintLine("Starting Yarp Network");
		yarp::os::Network yarp;
		PrintLine("...done!");
		
		PrintLine("Configuring Resource Finder");
		yarp::os::ResourceFinder rf;
		rf.setVerbose(true);
		rf.setDefaultContext(context);    
		rf.setDefaultConfigFile(config);    
		rf.configure(policy,argc,argv);
		PrintLine("...done!");

		PrintLine("Starting Grasp Module");
		GraspModule module; 
		module.runModule(rf);
	}

	catch (std::exception& e) {
		PrintLine(e.what());
	}

	catch (...) {
		PrintLine("Unknown exception");
	}

	return 0;
}


