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
 * @file ThreadFactory.cpp
 * @brief Factory method for grasp threads.
 */

#include "ThreadFactory.h"
#include "IcubThread.h"

using namespace darwin::grasp;

/**
  * @brief Creates a iCub/Parallel gripper grasping behaviour
  *
  * This function creates a new grasping thread according to the vocab given as parameter, each thread class is a different behaviour. The function can create a stub thread if the boolean is set to true. Adding a new case can be needed if there is a need to change the FSM internal structure or, maybe, if a kinematics strictly different than the humanoid one is used (e.g. barrett hand). In this case the internal infrastructure of the thread (the FSM) needs to be reprogrammed.
  *
  * @param rtype A vocab, either \a icub or another robot of your choise such as \a puma, stating the will of instantiating a grasping thread for the iCub or the parallel industrial gripper
  * @param stub A boolean stating the will of instantiation a stub version of the threads (stub==true) useful for integration tests.
  */

GraspModuleThread* ThreadFactory::create(const RobotType& rtype,bool stub) {

	GraspModuleThread* thr = 0;

	switch (rtype) {
	//case PUMA:
		//add your robot here
	//	break;
	case ICUB:
		if (stub) {
			thr = new IcubStub();
		}
		else {
			thr = new IcubThread();
		}
		break;
	default:
        throw std::runtime_error("Grasp Module ThreadFactory: Unknown RobotType");
		break;
	}

	return thr;
}
