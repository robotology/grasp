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
 * @file GraspModule.h
 * @brief The Grasp Module
 */

#pragma once

#include <yarp/os/ConnectionReader.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
 
//within project includes 
#include "Vocabs.h"
#include "PortMap.h"
#include "GraspModuleThread.h"

namespace darwin {
namespace grasp {

class GraspResultWriter {//abstract interface class, callback from thread to module
public:
	virtual bool write(GraspResult&) = 0;
};

/**
  * \author Giuseppe Cotugno
  * \author Kris De Meyer
  *
  * This class is the infrastructure for the grasper module, it basically takes care of all the YARP related operations.
  *
  */

class GraspModule : public yarp::os::RFModule, public yarp::os::PortReader, public GraspResultWriter {
public:

	/**
    *  Constructor
	*/
	GraspModule();

	/**
    *  Destructor
	*/
	virtual ~GraspModule();

    /**
    *  configure the ports and parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf); 
   
    /**
    *  interrupt, e.g., the ports 
    */
    bool interruptModule();                    

    /**
    *  close and shut down the grasp module
    */
    bool close();

    /**
    *  to respond through rpc port
    * @param command reference to bottle given to rpc port of module, alongwith parameters
    * @param reply reference to bottle returned by the rpc port in response to command
    * @return bool flag for the success of response else termination of module
    */
    bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply);

    /**
    *  The functionality of the module
    */ 
    bool updateModule();

	/** 
	 * Callback function to read incoming commands on the command port
	 * @return bool flag for the success of reading
	 */
	virtual bool read(yarp::os::ConnectionReader& connection);
	
	/**
	*  Write the grasp outcome back to the module
	*/
	virtual bool write(GraspResult& outcome);

private:

	// the rpc command ports for yarp and darwin commands
	PortMap<yarp::os::RpcServer> _commandPorts;

	// pointer to a new thread to be created and started in configure() and stopped in close()
    GraspModuleThread* _graspThread;  

	//flag that determines whether we're running under control of the yarp manager or not
	bool _yarpmanager;

	//flag that determines whether we block the sending module or reply with an immediate ACK message
	bool _blocking;

	//state machine flags
	bool _idle;
	bool _quit;
	bool _done;

	//a lock protecting access to the state flags
    yarp::os::Semaphore _stateLock;

	//incoming grasp commands (from Observer module)
	GraspCommand _command;

	//grasp result coming from thread
	GraspResult _outcome;
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

