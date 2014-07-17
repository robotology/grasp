/* Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori
 * email:   ilaria.gori@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found in the file LICENSE located in the
 * root directory.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
@ingroup robotology
\defgroup handIKModule handIKModule

A module that, given three 3D points and their normals, find the configuration of the
hand to make thumb, index and middle reach those three points. For further information:

I.Gori, U. Pattacini, V. Tikhanoff, G. Metta
Three-Finger Precision Grasp on Incomplete 3D Point Clouds.
In Proceedings of IEEE International Conference on Robotics and Automation (ICRA), 2014.

\section intro_sec Description 
This module, given triplet of 3D points, returns the configuration of the hand in terms
of joints position along with the end-effector position and orientation, so that thumb,
index and middle fingers reach those three points.

\section rpc_port Commands:

The commands sent as bottles to the module port /<modName>/rpc
are described in the following:

<b>IK</b> \n
format: [IKparam center (x y z) dim (x y z) c1 (x y z) c2 (x y z) c3 (x y z)
n1 (x y z) n2 (x y z) n3 (x y z) rot (r1 r2 r3 r4 r5 r6 r7 r8 r9)] \n
action: param can be 1, 2, 3 or 4. For this module it doesn't make any difference, but
it is useful for the precision-grasp module. c is the center of the object, dim represents
the dimension of the object, found using minimumBoundingBox, c1, c2 and c3 are the positions
of the points of the triplet, n1 n2 and n3 are the normals, rot is the rotation matrix
betwee the object reference frame and the robot reference frame.

\section lib_sec Libraries 
- YARP libraries. 
- \ref IPOPT library

\section portsc_sec Ports Created 

- \e /<modName>/rpc remote procedure call. It always replies something.
- \e /<modName>/<hand>/out this is the port that replies with the solution found by the
    module. It replies with the following format: [hand h cost c ee (x y z) or (x y z a)
    joints (j1 j2 j3 j4 j5 j6 j7 j8) combination (a b c)]. Hand is the hand for which
    the inverse kinematics problem has been solved. cost is the best value of the
    objective function. ee is the end effector position and or is its orientation.
    joints represent the joint angles, and combination tells you which point has to be 
    reached by the thumb, which from the index and which from the middle finger.

\section parameters_sec Parameters 
The following are the options that are usually contained 
in the configuration file:

--name \e name
- specify the module name, which is \e handIKModule by 
  default.

--robot \e hand
- specify the hand for which the problem is being solved.

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
**/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/math.h>
#include "handIKModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    
    if (!yarp.checkNetwork())
        return -1;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("handIK/conf");
    rf.setDefaultConfigFile("contactPoints_fitness1.ini");
    rf.configure("ICUB_ROOT",argc,argv);
    
    HandIKModule mod;
    mod.runModule(rf);

    return 0;
}


