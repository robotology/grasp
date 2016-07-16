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

#ifndef __ORIENTATION_THREAD_H__
#define __ORIENTATION_THREAD_H__

#include <cmath>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

class OrientationThread : public yarp::os::RateThread
{
private:

    int nAngles;
    int currentContext;
    double bestManip;
    bool done;
    bool work;
    bool noResult;
    std::string hand;

    yarp::dev::PolyDriver dCtrl;
    yarp::dev::ICartesianControl *iCtrl;
    yarp::dev::PolyDriver robotArm;
    yarp::dev::PolyDriver robotTorso;
    yarp::dev::IControlLimits *limTorso, *limArm;

    iCub::iKin::iCubArm *arm;
    iCub::iKin::iKinChain *chain;

    yarp::sig::Vector eePos;
    yarp::sig::Vector px;
    yarp::sig::Vector py;
    yarp::sig::Vector pointNormal;
    yarp::sig::Vector center;
    yarp::sig::Vector biggestAxis;
    yarp::sig::Vector thetaMin, thetaMax;
    yarp::sig::Matrix bestOrientation;
    yarp::sig::Vector od;
    yarp::sig::Vector q, q0;
    yarp::sig::Vector xdhat, odhat;
    yarp::sig::Vector ones;
    yarp::sig::Matrix Jacobian, mulJac;
    yarp::sig::Vector angles;

    void getAngles(yarp::sig::Vector &angles, int factor);
    bool normalDirection(std::string &hand, yarp::sig::Vector &normal);

public:

    OrientationThread();
    ~OrientationThread() {};

    bool open(std::string &name, std::string &arm, std::string &robot, int &nAngles);
    void close();
    bool checkDone();
    void setInfo(yarp::sig::Vector &eePos, yarp::sig::Vector &px, yarp::sig::Vector &py, yarp::sig::Vector &pointNormal, yarp::sig::Vector &center, yarp::sig::Vector &biggerAxis);
    void getBestManip(double &manip, yarp::sig::Matrix &orientation);
    bool getResult();
    void threadRelease();
    void run();
    void reset();
    void preAskForPose();
    void postAskForPose();

};

#endif
