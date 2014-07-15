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

