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


