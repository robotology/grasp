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
 * @file VocabDefinitions.h
 * @brief Definitions for vocabularies that are used as content in TypeSafeBottle messages
 */

#pragma once

#include <yarp/os/Vocab.h>

namespace darwin {
	namespace msg {

/**
 * Grip commands for the Puma platform
 * Sent from GraspModule to RobotController
 */
enum GripCommandType {
	GRIP_OPEN = VOCAB4('o','p','e','n'),
	GRIP_CLOSE = VOCAB4('c','l','o','s'),
	GRIP_STOP = VOCAB4('s','t','o','p'), //only for simulator
	GRIP_EXIT = VOCAB4('e','x','i','t')  //only for simulator
};

/**
 * Identifiers of grasp end effectors
 * Sent from the Observer to the GraspModule
 */
enum GraspEffectorType {
	GRASP_LEFT = VOCAB4('l','h','a','n'),
	GRASP_RIGHT = VOCAB4('r','h','a','n')
};

/**
 * Identifiers of the different grasp types
 * Sent from the Observer to the GraspModule
 */
enum GraspTypeType {
	GRASP_TYPE_UNKNOWN = 0,
    GRASP_POWER = VOCAB3('p','o','w'),
	GRASP_PINCH = VOCAB3('p','i','n'),
	GRASP_RELEASE = VOCAB3('r','e','l')
};

/**
 * Outcome of a grasp action
 * Sent from the GraspModule to the Observer
 * and (on the PROF simulator) from GraspModule to RobotController
 */
enum GraspResultType {
	GRASP_UNKNOWN = 0,
	GRASP_INTERRUPT = VOCAB4('i','r','p','t'),
	GRASP_ACK = VOCAB3('a','c','k'),
	GRASP_SUCC = VOCAB4('s','u','c','c'),
	GRASP_FAIL = VOCAB4('f','a','i','l')
};

} //end namespace msg
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

