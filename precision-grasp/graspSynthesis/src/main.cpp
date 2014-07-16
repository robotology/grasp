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
\defgroup precision-grasp precision-grasp

A module that, given a 3D point cloud representing an object, finds the best triplet to perform
a three-finger precision grasp successfully. 

\section intro_sec Description 
This module, given a set of 3D points, returns the best triplet of 3D points on the basis of
stability and feasibility properties. The search over the point cloud is executed by means of
a discrete particle swarm optimization algorithm. The triplet is then evaluated by the handIKModule, 
which returns a feasible configuration of the hand in terms of joint positions, as well as the 
position and the orientation of the end-effector. This information can successively be fed to
a module that actually executes the grasp. These information are also sent to some matlab scripts
that are inside the app/matlab folder. This module usually takes the 3D point cloud from the 
object-reconstruction module.

\section rpc_port Commands:

The commands sent as bottles to the module port /<modName>/rpc
are described in the following:

<b>HELP</b> \n
format: [help] \n
action: a list of all the possible commands to send to the rpc port is printed.

<b>SET_VISUALIZATION</b> \n
format: [set visualization param] \n
action: if param is set to on, a 3D viewer containing the reconstructed point cloud along with
the chosen end-effector position and orientation is depicted. Otherwise this functionality is
disabled.

<b>SET_WINDOW</b> \n
format: [set param1 param2] \n
action: param1 can be x, y, w or h, and it refers respectively to the x position, the y position,
the width and the height of the 3D viewer. param2 has to be set to the value that is wanted to 
assign.

<b>SET_OFFSET</b> \n
format: [set param1 x y z] \n
action: param1 can be offsetL, for the left arm, or offsetR, for the right arm. It sets an offset
on the corresponding arm (represented by x, y and z), in case there is some error in the mapping
between the eye and the arm.

<b>SET_FILTER</b> \n
format: [set filter param] \n
action: param can be set to on or off. It is usually set to on, as the stereo vision library
provides noisy point clouds. A statistical outlier removal filter is usually needed to obtain
clearer point clouds.

<b>SET_WRITE</b> \n
format: [set write param] \n
action: param can be set to on or off. If it is set to on, the current point cloud is saved
in the path specified in the config file.

<b>BLOCK</b> \n
format: [block param] \n
action: param can be set to right or left. It is useful if it is wanted to force the iCub
to use a specific arm. The arm can be unblocked using the command UNBLOCK.

<b>UNBLOCK</b> \n
format: [unblock param] \n
action: param can be set to right or left. It serves to unblock an arm that was previously
blocked using the command BLOCK.

<b>GRASP</b> \n
format: [grasp (x y)] "wait" \n
action: the algorithm runs to find a good triplet and a good configuration, then sends
such configuration to the actionsRenderingEngine module, which executes the grasp. If the optional
parameter wait is present, the grasp is computed and sent to matlab, but it is not executed. If
then the user wants to execute the grasp, he has to use the command GO. Otherwise, to reset
the computation, he can use the command DONT.

<b>GO</b> \n
format: [go] \n
action: if the computation of the grasp was successful but the grasp has not been executed
yet because the wait command has been added to GRASP, go makes the robot execute the grasp.

<b>DONT</b> \n
format: [dont] \n
action: it resets all the information that has been previously computed. It is usually used
when a grasp (x y) wait command has been sent, and the computed grasp is not satisfactory.

<b>ISGRASPED</b> \n
format: [isGrasped] \n
action: returns ACK if the grasp has been successfully executed by ARE, otherwise returns NACK.

\section lib_sec Libraries 
- YARP libraries. 
- \ref objects3D library
- \ref minimumBoundingBox library
- \ref forceClosure library
- \ref OpenCV library
- \ref Point Cloud Library.

\section portsc_sec Ports Created 

- \e /<modName>/rpc remote procedure call. It always replies something.
- \e /<modName>/are/cmd:o this is the port through which end-effector position
    and orientation are commanded to ARE to execute the grasp.
- \e /<modName>/toMatlab port that allows sending the hand configuration to some
    matlab scripts, that then depict the position of the hand over the object.
- \e /<modName>/ik1r:o /<modName>/ik2r:o /<modName>/ik3r:o /<modName>/ik4r:o the
    ports that communicate with the inverse kinematics module for the right hand.
- \e /<modName>/ik1l:o /<modName>/ik2l:o /<modName>/ik3l:o /<modName>/ik4l:o the
    ports that communicate with the inverse kinematics module for the left hand.
- \e /<modName>/mesh:i this is the port where the module reads the point cloud.
- \e /<modName>/reconstruction the module sends to this port a pixel belonging
    to the object to be segmented and reconstructed.
- \e /<modName>/depth2kin:o this is the port that asks depth2kin the 3D point
    computed taking into account the errors between the arm and the eye.

\section parameters_sec Parameters 
The following are the options that are usually contained 
in the configuration file:

--name \e name
- specify the module name, which is \e precision-grasp by 
  default.

--robot \e robot
- specify the robot that has to be used. It is icub by 
  default.

-- radiusSearch \e radiusSearch
- double representing the area of the neighborhood to be searched to compute
  surface normals. It is usually set to the area of the robot's palm.

-- path \e path
- in case the user wants to run the algorithm in simulation using an already
  reconstructed 3D point cloud, he can put a .ply file representing such cloud
  in the path directory.

-- fromFile \e fromFile
- if this parameter is set to true, the algorithm runs in simulation using a .ply file
  that contains the point cloud to be evaluated, which is in the "path" dir.

-- x \e x
- x position of the window that depicts the point cloud.

-- y \e y
- y position of the window that depicts the point cloud.

-- w \e w
- width of the window that depicts the point cloud.

-- h \e h
- height of the window that depicts the point cloud.

-- outputFile \e outputFile
- the folder where point clouds will be saved.

-- limit_finger_max \e limit_finger_max
- maximum distance between two fingers. Set to 0.08 by default.

-- limit_finger_min \e limit_finger_min
- minimum distance between two fingers. Set to 0.02 by default.

-- sampling \e sampling
- parameter to sample the point cloud through a voxel grid.

-- alpha \e alpha
- friction coefficient.

-- phi_p \e phi_p
- parameter for particle swarm optimization related to the best position
    reached from a particle over time.

-- phi_g \e phi_g
- parameter for particle swarm optimization related to the best position
    ever reached from the particles.

-- iterations \e iterations
- number of iterations for particle swarm optimization.

-- hand_area \e hand_area
- maximum area covered by thumb, index and middle fingers.

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
**/

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

