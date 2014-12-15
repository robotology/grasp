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
 * @file PortMap.h
 * @brief A container for ports, providing safe opening and closing functionality
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes
#include <map>
#include <stdexcept>

//yarp includes
#include <yarp/os/Contactable.h>

//local includes
#include "PrintFunctions.h"

namespace darwin {
namespace grasp {

//class declaration
template<class T>
class PortMap {
public:
	virtual ~PortMap();
	T& operator[](int);
	void open(int,const char*);
	void interrupt();
	void resume();
	void close();
	void clear();
private:
	std::map<int,T*> _ports;
};

//template class function definitions, need to include here to avoid linking problems
template<class T>
PortMap<T>::~PortMap() {
	close();
	clear();
}

template<class T>
void PortMap<T>::open(int key,const char* name) {
    typename std::map<int,T*>::iterator it = _ports.find(key);
	if (it != _ports.end()) {
        throw std::runtime_error("PortMap::open: duplicate key when adding port to map");
	}
	T* newport = new T;
	PrintDebugLine(yarp::os::ConstString("Opening Port: ") + name);
	if (!newport->open(name)) {
        throw std::runtime_error(std::string(yarp::os::ConstString("Failed to open port ").c_str()) + std::string(name));
	}
	_ports[key] = newport;
}

template<class T>
T& PortMap<T>::operator[](int key) {
    typename std::map<int,T*>::iterator it = _ports.find(key);
	if (it == _ports.end()) {
        throw std::runtime_error("PortMap:operator[]: port does not exist");
	}
	return *(it->second);
}

template<class T>
void PortMap<T>::interrupt() {
    for(typename std::map<int,T*>::iterator it = _ports.begin();it != _ports.end();++it) {
		PrintDebugLine(yarp::os::ConstString("Interrupting Port: ") + it->second->getName());
		it->second->interrupt();
	}
}

template<class T>
void PortMap<T>::resume() {
    for(typename std::map<int,T*>::iterator it = _ports.begin();it != _ports.end();++it) {
		PrintDebugLine(yarp::os::ConstString("Resuming Port: ") + it->second->getName());
		it->second->resume();
	}
}

template<class T>
void PortMap<T>::close() {
    for(typename std::map<int,T*>::iterator it = _ports.begin();it != _ports.end();++it) {
		PrintDebugLine(yarp::os::ConstString("Closing Port: ") + it->second->getName());
		it->second->close();
	}
}

template<class T>
void PortMap<T>::clear() {
    for(typename std::map<int,T*>::iterator it = _ports.begin();it != _ports.end();++it) {
		if (it->second) {
			PrintDebugLine(yarp::os::ConstString("Deleting Port: ") + it->second->getName());
			delete it->second;
			it->second = 0;
		}
	}
	_ports.clear();
}


} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------

