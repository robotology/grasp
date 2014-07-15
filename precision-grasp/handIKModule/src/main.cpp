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


