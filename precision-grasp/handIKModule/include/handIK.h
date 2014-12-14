/* Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini
 * email:   ugo.pattacini@iit.it
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


#ifndef __HANDIK_H__
#define __HANDIK_H__

#include <deque>
#include <yarp/sig/all.h>
#include <iCub/iKin/iKinFwd.h>


/****************************************************************/
struct HandIK_Problem
{
    iCub::iKin::iCubFinger thumb;
    iCub::iKin::iCubFinger index;
    iCub::iKin::iCubFinger middle;

    int nJoints;
    int nVars;
    int nFingers;
    std::string hand;
    std::deque<yarp::sig::Vector> normalDirs;       // to be given as list of 4x1 homogeneous unity vectors
    std::deque<yarp::sig::Vector> contactPoints;    // to be given as list of 4x1 homogeneous vectors
    yarp::sig::Vector             dimensions;       // to be given as 3x1 vector

    HandIK_Problem(const std::string &_hand="right", const int fingers=2) :
                   hand(_hand),                   
                   nFingers(fingers),
                   thumb(_hand+"_thumb"),
                   index(_hand+"_index"),
                   middle(_hand+"_middle")
    {
        nJoints=(nFingers==2)?6:8;
        nVars=3+3+nJoints;
        dimensions.resize(3,0.0);
    }
};


/****************************************************************/
struct HandIK_Variables
{    
    yarp::sig::Vector xyz_ee;
    yarp::sig::Vector rpy_ee;
    yarp::sig::Vector joints;
    int nFingers;
    double cost_fun;

    HandIK_Variables(const int fingers=2) : nFingers(fingers)
    {
        xyz_ee.resize(3,0.0);
        rpy_ee.resize(3,0.0);
        joints.resize((nFingers==2)?6:8,0.0);
        cost_fun=0.0;
    }

    void print();
};


/****************************************************************/
class HandIK_Solver
{
protected:
    HandIK_Problem &problem;
    HandIK_Variables guess;

public:
    HandIK_Solver(HandIK_Problem &_problem) : problem(_problem) { }
    bool setInitialGuess(const HandIK_Variables &_guess);
    bool solve(HandIK_Variables &solution);
};

 
#endif


