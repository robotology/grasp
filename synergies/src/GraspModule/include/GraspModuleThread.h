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
 * @brief Interface class for threads implementing grasp behaviour for different robotic platforms
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes

//yarp includes
#include <yarp/os/Thread.h>
#include <yarp/os/ResourceFinder.h>

//local includes
#include "MessageFormats/GraspMessages.h"

namespace darwin {
namespace grasp {

//pre-declaration
class GraspResultWriter;

class GraspModuleThread : public yarp::os::Thread {
public:
    /**
     * @brief Destructor
     */
	virtual ~GraspModuleThread() {}

	/**
    *  configure the ports and parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    virtual bool configure(yarp::os::ResourceFinder& rf) = 0;

    /**
    *  Initialises the thread
    */
    virtual bool threadInit() = 0;

    /**
    *  Correctly releases the thread
    */
    virtual void threadRelease() = 0;

	/**
	*  Main function running the thread
	*/
	virtual void run() = 0;

    /**
    *  On stopping of the thread
    */
    virtual void onStop() = 0;

    /**
     * @brief Suspend the processing of the module
     */
    virtual void suspend() = 0;

    /**
     * @brief Resume the processing of the module
     */
    virtual void resume() = 0;

    /**
     * @brief Visualization suspend method
     */
    virtual void visOff() = 0;

    /**
     * @brief Visualization resume method
     */
    virtual void visOn() = 0;

    /**
     * @brief function that test the network and feature of the thread
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    virtual bool test() = 0;

	/**
	 * @brief pass a GraspCommand message that has been received by the command port
	 * @param gc the message received by the command port from the observer module
	 */
	virtual bool write(darwin::msg::GraspCommand& gc) = 0;

	/**
	 * @brief pass a callback function for the outcome to the thread
	 * @param writer the callback function object
	 */
	virtual void setResultWriter(GraspResultWriter& writer) = 0;
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

