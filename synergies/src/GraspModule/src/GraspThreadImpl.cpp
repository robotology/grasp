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
 * @file GraspThreadImpl.cpp
 * @brief Base class for grasp threads. Implements common functionality (suspend, resume etc).
 */

//std includes
#include <stdexcept>

//yarp includes
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>

//common includes
#include "MessageFormats/VocabDefinitions.h"

//local includes
#include "Util.h"
#include "PrintFunctions.h"
#include "Vocabs.h"
#include "ConfigMessages.h"
#include "GraspThreadImpl.h"
#include "GraspModule.h"

using namespace std;
using namespace yarp::os;
using namespace darwin::grasp;

template<class InPort,class OutPort>
GraspThreadImpl<InPort,OutPort>::GraspThreadImpl() : _inPorts(),_outPorts(),_outcomePorts(),_stateLock(),_command(),_outcome() {
	_resultWriter = 0;
}

template<class InPort,class OutPort>
GraspThreadImpl<InPort,OutPort>::~GraspThreadImpl() {
	
}

template<class InPort,class OutPort>
bool GraspThreadImpl<InPort,OutPort>::configure(ResourceFinder& rf) {
    
	PrintDebugLine("===========================");
	PrintDebugLine("Configuring GraspThreadImpl");
	PrintDebugLine("===========================");

	//check and get the top level parameter settings
	if (!rf.check("module")) {
        throw runtime_error("GraspThreadImpl::configure: missing module name");
	}
	ConstString module = rf.find("module").asString();
	bool yarpmanager = true;
	if (rf.check("yarpmanager")) {
        yarpmanager = rf.find("yarpmanager").asInt()?true:false;
	}

	//check and get the inports and outports from the ResourceFinder
	if (!rf.check("effectors")) {
        throw runtime_error("GraspThreadImpl::configure: missing effectors element");
	}
	if (!rf.check("INPORTS")) {
        throw runtime_error("GraspThreadImpl::configure: missing INPORTS section");
	}
	if (!rf.check("OUTPORTS")) {
        throw runtime_error("GraspThreadImpl::configure: missing OUTPORTS section");
	}
	
	Bottle& effectors = rf.findGroup("effectors");
	Bottle& inports = rf.findGroup("INPORTS"); 
	Bottle& outports = rf.findGroup("OUTPORTS");

	if (!inports.check("measure")) {
        throw runtime_error("GraspThreadImpl::configure: missing INPORTS::measure section");
	}
	if (!outports.check("command")) {
        throw runtime_error("GraspThreadImpl::configure: missing OUTPORTS::command section");
	}
	if (!outports.check("outcome")) {
        throw runtime_error("GraspThreadImpl::configure: missing OUTPORTS::outcome section");
	}

	Bottle& measureports = inports.findGroup("measure");
	Bottle& commandports = outports.findGroup("command");
	Bottle& outcomeports = outports.findGroup("outcome");

	PrintDebugLine(ConstString("Effectors: ") + effectors.tail().toString());
	PrintDebugLine(ConstString("Measure ports: ") + measureports.tail().toString());
	PrintDebugLine(ConstString("Command ports: ") + commandports.tail().toString());
	PrintDebugLine(ConstString("Outcome ports: ") + outcomeports.tail().toString());

	if (effectors.size() != measureports.size()) {
        throw runtime_error("GraspThreadImpl::configure: size mismatch between effectors and INPORTS::measure");
    }
	if (effectors.size() != commandports.size()) {
        throw runtime_error("GraspThreadImpl::configure: size mismatch between effectors and OUTPORTS::command");
    }
	if (outcomeports.size() != 2) {
        throw runtime_error("GraspThreadImpl::configure: wrong number of outcome ports");
	}

	//open the ports
	for(int i=1;i<effectors.size();++i) {
		ConstString portname = get_port_name(module,measureports.get(i).asString());
		_inPorts.open(get_key(effectors,i),portname.c_str());
	}
	for(int o=1;o<effectors.size();++o) {
		ConstString portname = get_port_name(module,commandports.get(o).asString());
		_outPorts.open(get_key(effectors,o),portname.c_str());
	}
	_outcomePorts.open(OUTC,get_port_name(module,outcomeports.get(1).asString()).c_str());

	//set up the connections, but only if the yarp manager is not managing the connections
	if (!yarpmanager) {
		PrintDebugLine("Setting up network connections manually");
		//check and get the connect group
		if (!rf.check("CONNECT")) {
            throw runtime_error("GraspThreadImpl::configure: missing CONNECT section");
		}
		//use a class generated from XML
		PrintDebugLine(ConstString("GraspModule CONNECT expected: "));
		PrintDebugLine(ConnectStructure().toString());
        ConnectStructure connectstruct = *static_cast<ConnectStructure*>(rf.findGroup("CONNECT").get(1).asList());
		PrintDebugLine(ConstString("GraspModule CONNECT read: ")); 
		PrintDebugLine(connectstruct.toString());
		//we need gripin, gripout and outcome fields
		ConnectField& gripin = connectstruct.gripin();
		ConnectField& gripout = connectstruct.gripout();
		ConnectField& outcome = connectstruct.outcome();

		//connect the ports
		for(int i=1;i<effectors.size();++i) {
			ConstString oname = gripin[i-1];
			ConstString iname = _inPorts[get_key(effectors,i)].getName();
			PrintDebugLine(ConstString("IN Network connections: ") + oname + ConstString(" / ") + iname);
			Network::connect(oname.c_str(),iname.c_str());
		}
		for(int o=1;o<effectors.size();++o) {
			ConstString oname = _outPorts[get_key(effectors,o)].getName();
			ConstString iname = gripout[o-1];
			PrintDebugLine(ConstString("OUT Network connections: ") + oname + ConstString(" / ") + iname);
			Network::connect(oname.c_str(),iname.c_str());
		}
		if (outcome.size()) {
			ConstString iname = outcome[0];
			ConstString oname = _outcomePorts[OUTC].getName();
			PrintDebugLine(ConstString("OUTCOME Network connections: ") + oname + ConstString(" / ") + iname);
			Network::connect(oname.c_str(),iname.c_str());
		}
	}

    return true;
}

template<class InPort,class OutPort>
bool GraspThreadImpl<InPort,OutPort>::threadInit() {
	//set the state flags in a safe way
	_stateLock.wait();
	_vis = true;
	_idle = false;//true;  //TODO revert
	_new = false;
	_done = false;
	_stateLock.post();
	return true;
};

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::suspend() {
	_stateLock.wait(); 
	_idle = true; 
	_stateLock.post();
};

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::resume() {
	_stateLock.wait(); 
	_idle = false; 
	_stateLock.post();
};

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::visOff(){
    _stateLock.wait();
	_vis = false;
	_stateLock.post();
}

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::visOn(){
	_stateLock.wait();
	_vis = true;
	_stateLock.post();
}

template<class InPort,class OutPort>
bool GraspThreadImpl<InPort,OutPort>::test(){
	return true;
}

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::threadRelease() {
	_stateLock.wait();
	_inPorts.close();
	_outPorts.close();
	_outcomePorts.close();
	_inPorts.clear();
	_outPorts.clear();
	_outcomePorts.clear();
	_stateLock.post();
}

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::onStop() {
	
}

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::setResultWriter(GraspResultWriter& writer) {
	_resultWriter = &writer;
}

template<class InPort,class OutPort>
bool GraspThreadImpl<InPort,OutPort>::write(GraspCommand& c) {
	_stateLock.wait();

	  _command = c;

	  PrintLine(c.toString());
	  PrintLine(_command.toString());

	PrintDebugLine("GraspThreadImpl::write: received GraspCommand from GraspModule: ");
    PrintDebugLine(_command.toString());
	_new = true;
	_stateLock.post();
	return true;
}

template<class InPort,class OutPort>
void GraspThreadImpl<InPort,OutPort>::handleGraspDone() {
	_stateLock.wait();
	PrintDebugLine(ConstString("GraspThread: passing GraspResult to GraspModule: ") + _outcome.toString());
	if (_outcome[0] == GRASP_SUCC || _outcome[0] == GRASP_FAIL) {
		_resultWriter->write(_outcome);
	}
	PrintLine(ConstString("GraspThread: writing GraspOutcome on OUTC port ") + _outcome.toString());
	if (_outcomePorts[OUTC].getOutputCount()) {
		GraspResult& o = _outcomePorts[OUTC].prepare();
		o = _outcome;
		_outcomePorts[OUTC].write();
	}
	_outcome.clear();
	_done = false;
	_stateLock.post();
}








