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
 * @file Vocabs.h
 * @brief Vocabulary definitions for the Grasp Module
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//std includes
#include <iostream>
#include <string>
#include <map>

//yarp includes
#include <yarp/os/Vocab.h>

namespace darwin {
namespace grasp {

typedef std::map<std::string,int> ObjVocabMap;


enum IcubJoints{    //full set of joints as named in the iCub repo, from 0 to 16
    j_shoulder_pitch,
    j_shoulder_roll,
    j_shoulder_yaw,
    j_elbow,
    j_wrist_prosup,
    j_wrist_pitch,
    j_wrist_yaw,
    j_finger_add_abd,
    j_thumb_oppose,
    j_thumb_proximal,
    j_thumb_distal,
    j_index_proximal,
    j_index_distal,
    j_middle_proximal,
    j_middle_distal,
    j_pinky_ring,
    IcubJointsNum
};


enum IcubHandJoints{    //set of joint names as locally indexed in KCL module, from 0 to 9 (add 6 to each of those to get indexing according to iCub repo)
    finger_add_abd,
    thumb_oppose,
    thumb_proximal,
    thumb_distal,
    index_proximal,
    index_distal,
    middle_proximal,
    middle_distal,
    pinky_ring,
    JHandsNum
};

enum IcubThumbPos {
    THUMB_SIDE = VOCAB2('s','d'),
    THUMB_UP = VOCAB2('u','p'),
    PINCH_GRIP = VOCAB3('p','i','n'),
    THUMB_NUM = 2
};

enum GraspTypeIndustrialSimulator {
	GRASP_SIM_CLOSE_FAIL = VOCAB4('s','c','l','f'),
	GRASP_SIM_CLOSE_SUCC = VOCAB4('s','c','l','s')
};

enum RobotType {
	ICUB = VOCAB4('i','c','u','b')
//	PUMA = VOCAB4('p','u','m','a')	//add your robot here to be used in ThreadFactory
};

enum PortMapKey {
	YARP = VOCAB4('y','a','r','p'),
	DRWN = VOCAB4('d','r','w','n'),
	OUTC = VOCAB4('o','u','t','c')
};

enum SystemCommand {
	HELP = VOCAB4('h','e','l','p'),
	QUIT = VOCAB4('q','u','i','t'),
	TEST = VOCAB4('t','e','s','t'),
	FAIL = VOCAB4('f','a','i','l'),
	SUSPEND = VOCAB3('s','u','s'),
	RESUME = VOCAB3('r','e','s'),	
	VIS = VOCAB3('v','i','s'),
	OFF = VOCAB3('o','f','f'),
	ON = VOCAB2('o','n')
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------



