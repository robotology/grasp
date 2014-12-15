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
 * @file GraspModuleThread.h
 * @brief Base class for threads implementing grasp behaviour for different robotic platforms
 *
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//yarp includes
#include <yarp/os/Semaphore.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>

//local includes
#include "PortMap.h"
#include "GraspModuleThread.h"

namespace darwin {
namespace grasp {

template<class InPort,class OutPort>
class GraspThreadImpl : public GraspModuleThread {
public:
    /**
    * @brief Default Constructor
    */
    GraspThreadImpl();

    /**
     * @brief Destructor
     */
    virtual ~GraspThreadImpl();

	/**
    *  configure the ports and parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    virtual bool configure(yarp::os::ResourceFinder& rf);

    /**
    *  Initialises the thread
    */
    virtual bool threadInit();

    /**
    *  Correctly releases the thread
    */
    virtual void threadRelease();

    /**
    *  On stopping of the thread
    */
    virtual void onStop();

    /**
     * @brief Suspend the processing of the module
     */
    virtual void suspend();

    /**
     * @brief Resume the processing of the module
     */
    virtual void resume();

    /**
     * @brief Visualization suspend method
     */
    virtual void visOff();

    /**
     * @brief Visualization resume method
     */
    virtual void visOn();

	/**
     * @brief function that test the network and feature of the thread
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    virtual bool test();

	/**
	 * @brief pass a GraspCommand message that has been received by the command port
	 * @param gc the message received by the command port from the observer module
	 */
	virtual bool write(darwin::msg::GraspCommand& gc);

	/**
	 * @brief pass a callback function for the outcome to the thread
	 * @param writer the callback function object
	 */
	virtual void setResultWriter(GraspResultWriter& writer);

protected:
	//maps for the input and output ports, connecting to and from robot or posture control modules
	PortMap<InPort> _inPorts;
	PortMap<OutPort> _outPorts;
	PortMap<yarp::os::BufferedPort<GraspResult> > _outcomePorts;

	//the callback function object to pass GraspResults back to the GraspModule
	GraspResultWriter* _resultWriter;

	//the current grasp command
	GraspCommand _command;

	//the outcome of the grasp
	GraspResult _outcome;

	//state machine flags
	bool _idle; //suspends processing
	bool _vis;  //turns visualisation on and off
	bool _new;  //a new grasp command was received from the module
	bool _done; //pass the result of the grasp back to the module

	//a lock for any read/write operations that affect the state of the machine
	yarp::os::Semaphore _stateLock;

	//a function that deals with the case when the grasp is completed
	void handleGraspDone();
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

