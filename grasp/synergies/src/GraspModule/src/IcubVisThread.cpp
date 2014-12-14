// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C)2013  King's College London
 * Authors: Kris De Meyer, Giuseppe Cotugno, Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file IcubVisThread.cpp
 * @brief Implementation of the thread that represent image (see header IcubVisThread.h)
 */

//local includes
#include "Util.h"
#include "IcubVisThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace darwin {
	namespace grasp {


IcubVisThread::IcubVisThread() {

}

IcubVisThread::~IcubVisThread() {

}

bool IcubVisThread::configure(ResourceFinder& rf) {

	PrintDebugLine("=========================");
	PrintDebugLine("Configuring IcubVisThread");
	PrintDebugLine("=========================");

	//check and get the top level parameter settings
	if (!rf.check("module")) {
        throw runtime_error("IcubVisThread::configure: missing module name");
	}
	ConstString module = rf.find("module").asString();
	
	if (!rf.check("effectors")) {
        throw runtime_error("IcubVisThread::configure: missing effectors element");
	}
	if (!rf.check("VISPORTS")) {
        throw runtime_error("IcubVisThread::configure: missing VISPORTS section");
	}
	
	Bottle& effectors = rf.findGroup("effectors");
	Bottle& visports = rf.findGroup("VISPORTS"); 

	if (!visports.check("state")) {
        throw runtime_error("IcubVisThread::configure: missing VISPORTS::state section");
	}

	Bottle& stateports = visports.findGroup("state");
	
	PrintDebugLine(ConstString("Effectors: ") + effectors.tail().toString());
	PrintDebugLine(ConstString("State ports: ") + stateports.tail().toString());
	
	if (effectors.size() != stateports.size()) {
        throw runtime_error("IcubVisThread::configure: size mismatch between effectors and VISPORTS::state");
    }

	//open the ports
	for(int i=1;i<effectors.size();++i) {
		ConstString portname = get_port_name(module,stateports.get(i).asString());
		_visPorts.open(get_key(effectors,i),portname.c_str());
	}

	//reading base visualizationimages
    //TODO....

	PrintDebugLine("==================================");
	PrintDebugLine("Finished Configuring IcubVisThread");
	PrintDebugLine("==================================");

    return true;
}

bool IcubVisThread::threadInit() {
   return true;
}

void IcubVisThread::threadRelease() {
    _visPorts.close();
	_visPorts.clear();
}

void IcubVisThread::interrupt() {
    
}

void IcubVisThread::run() {
       
}

}
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
