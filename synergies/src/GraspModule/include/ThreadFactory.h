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
 * @file ThreadFactory.h
 * @brief Factory function to create threads specialised for each robotic platform
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes

//yarp includes
#include <yarp/os/ConstString.h>

//local includes
#include "Vocabs.h"
#include "GraspModuleThread.h"

namespace darwin {
namespace grasp {

class ThreadFactory {
public:
	static GraspModuleThread* create(const RobotType&,bool stub);
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------



