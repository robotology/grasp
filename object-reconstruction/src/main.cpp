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
\defgroup object-reconstruction object-reconstruction

A module to reconstruct in 3D a set of pixel and visualize the reconstruction. 

\section intro_sec Description 
This module, given a set of pixels, returns the point cloud along with the minimum enclosing 
bounding box. It usually employs the graphBasedSegmentation module, which provides all the pixels
belonging to a certain segment of the image, but it can be used with any other segmentation 
module that retrieves the list of pixels belonging to the object to be reconstructed.

\section rpc_port Commands:

The commands sent as bottles to the module port /<modName>/rpc
are described in the following:

<b>HELP</b> \n
format: [help] \n
action: it returns the list of the things to do to use the module properly.

<b>PIXEL</b> \n
format: [u v] \n
action: the pixel is given to the segmentation module, which responds with the pixels belonging
to the chosen segment.

<b>RECONSTRUCT</b> \n
format: [3Drec] "param1"\n
action: the set of pixels are projected in 3D and provided on the output port -- typically
/objectReconstr/mesh:o. The optional parameter "param1" can be set to "visualize"; in this case
the reconstructed cloud will also be displayed using Point Cloud Library library.

<b>GET</b> \n
format: [get point (u v)]\n
action: The pixel (u v) is projected in 3D and returned in the format (x y z).

<b>SET</b> \n
format: [set write param]\n
action: If param is set to on, the reconstructed point cloud will be written in the home context
    path folder. This functionality can be turned off by setting param to off.

\section lib_sec Libraries 
- YARP libraries. 
- \ref objects3D library
- \ref minimumBoundingBox library
- \ref stereo-vision library
- \ref OpenCV library
- \ref Point Cloud Library.

\section portsc_sec Ports Created 

- \e /<modName>/rpc remote procedure call. It always replies something.
- \e /<modName>/segmentation this is the port through which the pixel 
    belonging to the object is sent to the segmentation module, and the 
    set of pixels belonging to the object are returned.
- \e /<modName>/mesh:o this is the port where the reconstructed object
    is returned.

\section parameters_sec Parameters 
The following are the options that are usually contained 
in the configuration file:

--name \e name
- specify the module name, which is \e object-reconstruction by 
  default.

--robot \e robot
- specify the robot that has to be used. It is icub by 
  default.

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
**/

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <objectReconstr.h>

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        printf("No yarp network\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("object-reconstruction");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    ObjectReconstr mod;

    return mod.runModule(rf);
}

