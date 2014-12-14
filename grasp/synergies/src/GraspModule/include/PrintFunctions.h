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
 * @file PrintFunctions.h
 * @brief Helper functions to print messages to stdout
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes
#include <iostream>
#include <string>

//yarp includes
#include <yarp/os/ConstString.h>

namespace darwin {
namespace grasp {

inline void PrintLine(const yarp::os::ConstString& text) {
	std::cout << "GRSP> " << text << std::endl;
}

/*inline void PrintLine(const std::string& text) {//disabled to work with older yarp versions
	std::cout << "GRSP> " << text << std::endl;
}*/

inline void PrintLine(const char* text) {
	std::cout << "GRSP> " << text << std::endl;
}

inline void PrintDebugLine(const yarp::os::ConstString& text) {
#ifdef GRASPMODULE_PRINT_DEBUG
	std::cout << "DEBUG> " << text << std::endl;
#endif
}

/*inline void PrintDebugLine(const std::string& text) {//disabled to work with older yarp versions
#ifdef GRASPMODULE_PRINT_DEBUG
	std::cout << "DEBUG> " << text << std::endl;
#endif
}*/

inline void PrintDebugLine(const char* text) {
#ifdef GRASPMODULE_PRINT_DEBUG
	std::cout << "DEBUG> " << text << std::endl;
#endif
}

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------



