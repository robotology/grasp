#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "precisionGrasp.h"
#include <iCub/grasp/forceClosure.h>

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("precision-grasp");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    PrecisionGrasp mod;

    return mod.runModule(rf);
}

