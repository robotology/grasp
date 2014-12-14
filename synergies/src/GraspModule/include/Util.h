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
 * @file Util.h
 * @brief Utility functions
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes
#include <stdexcept>

//yarp includes
#include <yarp/os/ConstString.h>
#include <yarp/os/Bottle.h>

using namespace std;
using namespace yarp::os;

namespace darwin {
namespace grasp {

inline int get_key(Bottle& b,int index) {

	int key = -1;
	if (index >= b.size()) {
        throw runtime_error("Util::get_key: index out of bounds");
	}

	yarp::os::Value& v = b.get(index);
	if (v.isInt()) {
		key = v.asInt();
	}
	else if (v.isString()) {
		key = yarp::os::Value(v.asString(),true).asVocab();
	}
	else {
        throw std::runtime_error("Util::get_key: key for PortMap should have integer type");
	}
	return key;
}

inline ConstString get_port_name(const ConstString& module,const ConstString& port) {

	ConstString mslash = module;
	if (mslash != "") {
		mslash = ConstString("/") + module;
	}

	return mslash + ConstString("/") + port;
}

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------



