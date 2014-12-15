// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  King's College London
  * Author: Giuseppe Cotugno, Kris De Meyer
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
 * @file GraspModule.cpp
 * @brief The Grasp Module
 */

//yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

//local includes
#include "PrintFunctions.h"
#include "Vocabs.h"
#include "Util.h"
#include "GraspModule.h"
#include "ThreadFactory.h"
#include "ConfigMessages.h"
#include "MessageFormats/VocabDefinitions.h"
#include "MessageFormats/GraspMessages.h"

using namespace std;
using namespace yarp::os;
using namespace darwin::msg;
using namespace darwin::grasp;

/* 
 * Constructor
 */

GraspModule::GraspModule() : _commandPorts(),_command(),_outcome(),_stateLock() {
	_graspThread = 0;
	_yarpmanager = true;
	_blocking = false;
	_idle = false;
	_quit = false;
	_done = false;
}

/* 
 * Destructor
 */

GraspModule::~GraspModule() {
	close();
}

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 */

bool GraspModule::configure(yarp::os::ResourceFinder& rf) {
    
	PrintDebugLine("=======================");
	PrintDebugLine("Configuring GraspModule");
	PrintDebugLine("=======================");

	//check and get the top level parameter settings for the module
	if (!rf.check("module")) {
        throw runtime_error("GraspModule::configure: missing module name");
	}
	if (!rf.check("robot")) {
        throw runtime_error("GraspModule::configure: missing robot name");
	}
	if (!rf.check("stub")) {
        throw runtime_error("GraspModule::configure: missing stub flag");
	}
	if (!rf.check("blocking")) {
        throw runtime_error("GraspModule::configure: missing blocking flag");
	}

	ConstString module = rf.find("module").asString();
	ConstString robotname = rf.find("robot").asString();
	RobotType robot = static_cast<RobotType>(yarp::os::Value(robotname.c_str(),true).asVocab());
	bool stub = rf.find("stub").asInt()?true:false;
	_blocking = rf.find("blocking").asInt()?true:false;

	if (rf.check("yarpmanager")) {
        _yarpmanager = rf.find("yarpmanager").asInt()?true:false;
	}

	//set the name of the module
	setName(module.c_str());

	PrintDebugLine(ConstString("Module name: ") + module);
	PrintDebugLine(ConstString("Robot name: ") + robotname);
	PrintDebugLine(ConstString("Stub behaviour: ") + Value(stub).toString());
	PrintDebugLine(ConstString("Yarp manager: ") + Value(_yarpmanager).toString());
	PrintDebugLine(ConstString("Blocking grasp commands: ") + Value(_blocking).toString());

	//check and get the inports and outports from the ResourceFinder
	if (!rf.check("INPORTS")) {
        throw runtime_error("GraspModule::configure: missing INPORTS section");
	}
	if (!rf.check("OUTPORTS")) {
        throw runtime_error("GraspModule::configure: missing OUTPORTS section");
	}
	Bottle& inports = rf.findGroup("INPORTS"); 
	Bottle& outports = rf.findGroup("OUTPORTS");

	if (!inports.check("command")) {
        throw runtime_error("GraspModule::configure: missing INPORTS::command section");
	}

	Bottle& commandports = inports.findGroup("command");

	PrintDebugLine(ConstString("Command ports: ") + commandports.tail().toString());

        if (commandports.size() != 3) {     //TODO hardcoded parameter here - if input ports changes this must be manually changed
        throw runtime_error("GraspModule::configure: wrong number of command ports");
	}
	
	//open the command ports
	_commandPorts.open(YARP,get_port_name(module,commandports.get(1).asString()).c_str());
	_commandPorts.open(DRWN,get_port_name(module,commandports.get(2).asString()).c_str());

	//attach the YARP port to the respond function mechanism
	PrintDebugLine("Attaching YARP port");
    attach(_commandPorts[YARP]);

	//set up the connections, but only if the yarp manager is not managing the connections
	if (!_yarpmanager) {
		PrintDebugLine("Setting up network connections manually");
		if (!rf.check("CONNECT")) {
            throw runtime_error("GraspModule::configure: missing CONNECT section");
		}
		//use a class generated from XML
		PrintDebugLine(ConstString("GraspModule CONNECT expected: "));
		PrintDebugLine(ConnectStructure().toString());
        ConnectStructure connectstruct = *static_cast<ConnectStructure*>(rf.findGroup("CONNECT").get(1).asList());
		PrintDebugLine(ConstString("GraspModule CONNECT read: "));
		PrintDebugLine(connectstruct.toString());

		//we only need the "command" section here
		ConnectField& command = connectstruct.command();

		//there is only 1 command port we need to connect manually
		ConstString oname = command[0];
		ConstString iname = _commandPorts[DRWN].getName();
		PrintDebugLine(ConstString("Network connections: ") + oname + ConstString(" / ") + iname);
		Network::connect(oname.c_str(),iname.c_str());	
	}
    
    //create the thread and configure it using the ResourceFinder
    _graspThread = ThreadFactory::create(robot,stub);
    _graspThread->configure(rf);

	//pass a callback function to pass the GraspResult back to the module
	_graspThread->setResultWriter(*this);

    //set a callback function for the DRWN port
    PrintDebugLine("Setting Reader for DRWN port");
    _commandPorts[DRWN].setReader(*this);

    //now start the thread to do the work
    _graspThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool GraspModule::interruptModule() {
	_commandPorts.interrupt();
    return true;
}

bool GraspModule::close() {
	/*close and clean up the ports*/
	_commandPorts.interrupt();
    _commandPorts.close();
	_commandPorts.clear();
    /*clean up the thread*/
	if (_graspThread) {
		PrintDebugLine("Deleting GraspThread...");
		delete _graspThread;
		_graspThread = 0;
	}
    return true;
}

bool GraspModule::respond(const Bottle& command,Bottle& reply) {

	bool rec = true; //is the command recognized?
    
	_stateLock.wait();

	PrintLine(ConstString("GraspModule::respond: receiving command: ") + command.toString());
	reply.clear();

    switch (command.get(0).asVocab()) {
	case HELP:
		reply.addVocab(Vocab::encode("many"));
        reply.addString("Help: ");
        reply.addString("Implemented commands are:");
        reply.addString(" help    : to get help");
        reply.addString(" quit    : to quit the module");
        reply.addString(" ");
        reply.addString(" sus     : to suspend the processing");
        reply.addString(" res     : to resume  the processing");
        reply.addString(" ");
        reply.addString(" vis on  : to enable  the visualization");
        reply.addString(" vis off : to disable the visualization");
        reply.addString(" ");
        reply.addString(" test    : automatic test of the features of the module");
        reply.addString(" ");
		break;
	case QUIT:
		_graspThread->stop();
		_quit = true;
        reply.addString("Preparing to quit grasp module");
        break;
	case TEST:
		if (_graspThread->test()) {
            reply.addString("Test started");
		}
		else {
            reply.addString("Test not started");
		}
        break;
	case VIS:
        switch(command.get(1).asVocab()) {
		case ON:
			_graspThread->visOn();
			reply.addString("Visualisation ON");
			break;
		case OFF:
			_graspThread->visOff();
			reply.addString("Visualisation OFF");
			break;
		default:
			rec = false;
		}
        break;
	case SUSPEND:
		_idle = true;
		_graspThread->suspend();
        reply.addString("Suspending grasping");
        break;
	case RESUME:
		_idle = false;
		_graspThread->resume();
        reply.addString("Resuming grasping");
        break;
    default:
		reply.addString("Unknown command send to GraspModule");
    }    
	_stateLock.post();
	PrintLine(ConstString("GraspModule::respond: returning the reply: ") + reply.toString());
    return true;
}


bool GraspModule::updateModule() {

	bool returnval = true;

	_stateLock.wait();
	if (_idle) {
		PrintLine("GraspModule::updateModule: suspended.");
	}
	if (_quit) {
		PrintLine("GraspModule::updateModule: quitting.");
		//we've received a "quit" command over the yarp network, time to stop the Module
		returnval = false;
	}
	_stateLock.post();

	//true in all cases, except for when the "_quit" flag was set previously by the respond function
    return returnval;
}

bool GraspModule::read(ConnectionReader& connection) {

  //this callback function is the main handler for incoming information
  //it can run in both blocking and non-blocking mode

  bool returnval = false;
  if (!_quit) {//to make sure this function is not called anymore while we are quitting
      //we have a new command coming in
      returnval = _command.read(connection);

      if(_command.size()>0){

          PrintLine("GraspModule::read: received GraspCommand:");
          PrintLine(_command.toString());

          //until here the _command has the data for sure
          _graspThread->write(_command);
          if (returnval) {//we have successfully read a value
              if (!_blocking) {
                  //we're not blocking the sender, so prepare the ACK signal
                  _stateLock.wait();
                  _outcome.clear();
                  _outcome.add(GRASP_ACK);
                  _stateLock.post();
                }
              if (_blocking) {
                  //we're blocking the sender, so we will have to wait here until result is ready
                  _stateLock.wait();
                  _outcome.clear();
                  _outcome.add(GRASP_UNKNOWN);
                  _stateLock.post();
                  for(int i=0;i<2000;++i) {
                      //wait for about 200 seconds for the thread to report success or failure
                      Time::delay(0.1);
                      _stateLock.wait();
                      if (_done) {
                          _done = false;
                          i = 2000; //this means move to the end of the for loop
                        }
                      _stateLock.post();
                    }
                }
              //we now have 3 different situations:
              //when the grasping is non-blocking, _outcome contains GRASP_ACK
              //when the grasping timed-out, _outcome contains GRASP_UNKNOWN
              //when the grasping finished, _outcome contains the result set in the GraspModule::write function
              ConnectionWriter* pw = connection.getWriter();
              if (pw) {
                  PrintLine(ConstString("Writing GraspResult on DRWN port: ") + _outcome.toString());
                  _outcome.write(*pw);
                }
              else {
                  PrintLine(ConstString("Cannot write result on DRWN port: ") + _outcome.toString());
                }
            }
        }
    }
  return returnval;
}

bool GraspModule::write(GraspResult& o) {
	_stateLock.wait();
	_outcome = o;
	PrintLine(ConstString("GraspModule::write: received GraspResult from GraspModuleThread: ") + _outcome.toString());
	_done = true;
	_stateLock.post();
	return true;
}
