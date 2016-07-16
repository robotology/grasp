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
\defgroup power-grasp power-grasp

A module that, given a 3D point cloud representing an object, finds the best position
on the object where the hand should be placed to obtain a successful power grasp. For
further information:

I.Gori, U. Pattacini, V. Tikhanoff, G. Metta
Ranking the Good Points: A Comprehensive Method for Humanoid Robots to Grasp Unknown Objects
In Proceedings of IEEE International Conference on Advanced Robotics (ICAR), 2013.

\section intro_sec Description 
This module, given a set of 3D points, returns the best end-effector position and orientation
that will lead to a successful grasp. It commands directly to the actionsRenderingEngine
module, which is equipped with a dedicated function called powerGrasp. It usually takes the
3D point cloud from the object-reconstruction module.

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

<b>SET_TRAIN</b> \n
format: [set train param] \n
action: if param is set to on, the module runs in training modality; this means that points will
be chosen randomly and saved to a dedicated file for a later training stage.

<b>SET_TESTSVM</b> \n
format: [set testWithSVM param] \n
action: if param is set to on, the module uses the function learned through SVM, which associate
a curvature to the successful probability of the grasp. Usually this parameter is set to off,
as we already know the best curvature that should be chosen for the hand of the iCub.

<b>SET_OFFSET</b> \n
format: [set param1 x y z] \n
action: param1 can be offsetL, for the left arm, or offsetR, for the right arm. It sets an offset
on the corresponding arm (represented by x, y and z), in case there is some error in the mapping
between the eye and the arm.

<b>SET_MODALITY</b> \n
format: [set modality param] \n
action: param can be set to right, left, center or top. It imposes the corresponding area of the 
object from which candidate end-effector positions should be chosen.

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
action: the algorithm runs to find a good end-effector position and orientation, and sends
such pose to the actionsRenderingEngine module, which executes the grasp. If the optional
parameter wait is present, the grasp is computed and depicted but it is not executed. If
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
- \ref OpenCV library
- \ref Point Cloud Library.

\section portsc_sec Ports Created 

- \e /<modName>/rpc remote procedure call. It always replies something.
- \e /<modName>/are/cmd:o this is the port through which end-effector position
    and orientation are commanded to ARE to execute the grasp.
- \e /<modName>/mesh:i this is the port where the module reads the point cloud.
- \e /<modName>/reconstruction the module sends to this port a pixel belonging
    to the object to be segmented and reconstructed.
- \e /<modName>/depth2kin:o this is the port that asks depth2kin the 3D point
    computed taking into account the errors between the arm and the eye.

\section parameters_sec Parameters 
The following are the options that are usually contained 
in the configuration file:

--name \e name
- specify the module name, which is \e power-grasp by 
  default.

--robot \e robot
- specify the robot that has to be used. It is icub by 
  default.

--nAngles \e nAngles
- number of angles to be analyzed to choose the best end-effector orientation.

-- radiusSearch \e radiusSearch
- double representing the area of the neighborhood to be searched to compute
  surface normals. It is usually set to the area of the robot's palm.

-- numberOfBestPoints \e numberOfBestPoints
- number of the best points chosen with respect to their curvature. Those points
  will be successively evaluated to choose the end-effector orientation.

-- curvature \e curvature
- best curvature that probably leads to successful grasps given the robot's palm.

-- handSize \e handSize
- size of the hand from the thumb to the little.

-- fingerSize \e fingerSize
- size of the middle finger.

-- path \e path
- in case the user wants to run the algorithm in simulation using an already
  reconstructed 3D point cloud, he can put a .ply file representing such cloud
  in the path directory.

-- fromFile \e fromFile
- if this parameter is set to true, the algorithm runs in simulation using a .ply file
  that contains the point cloud to be evaluated, which is in the "path" dir.

-- useLearning \e useLearning
- if this parameter is set to true, the lssvm previously trained will be used. Usually this
  parameter is set to false, as we already know which is the best curvature for the iCub's  
  hand.

-- x \e x
- x position of the window that depicts the point cloud.

-- y \e y
- y position of the window that depicts the point cloud.

-- w \e w
- width of the window that depicts the point cloud.

-- h \e h
- height of the window that depicts the point cloud.

-- disableRight \e disableRight
- if the user wants to disable the right arm from the beginning (for instance if the robot has the 
  right arm broken, so the cartesian interface cannot be opened), this parameter should be present
  in the config file. If it is not present, the right arm will not be disabled.

-- disableLeft \e disableLeft
- if the user wants to disable the left arm from the beginning (for instance if the robot has the 
  left arm broken, so the cartesian interface cannot be opened), this parameter should be present
  in the config file. If it is not present, the left arm will not be disabled.

-- offsetR \e offsetR
- it is possible to set an offset to compensate the error between the right arm and the eye.

-- offsetL \e offsetL
- it is possible to set an offset to compensate the error between the left arm and the eye.

-- outputDir \e outputDir
- the folder where point clouds will be saved.

-- lssvm \e \lssvm
- a group of parameters as they are returned by the learningMachine library in icub-main.

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
**/

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "powerGrasp.h"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
        return 1;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("power-grasp");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    PowerGrasp mod;

    return mod.runModule(rf);
}

