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

#include "orientationThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

/************************************************************************/
OrientationThread::OrientationThread() : RateThread(20) 
{
    iCtrl=NULL;
    arm=NULL;
    done=true;
    work=false;
    noResult=true;
}

/************************************************************************/
bool OrientationThread::open(string &name, string &hand, string &robot, int &nAngles)
{
    this->nAngles=nAngles;
    this->hand=hand;

    string remoteName="/"+robot+"/cartesianController/"+hand;
    Property optCtrl;
    optCtrl.put("device","cartesiancontrollerclient");
    optCtrl.put("remote",remoteName.c_str());
    optCtrl.put("local",("/"+name+"/orientationThread/"+hand+"/cartesian").c_str());

    if (!dCtrl.open(optCtrl))
    {
        fprintf(stdout, "%s Cartesian Interface is not open\n", hand.c_str());
        return false;
    }

    dCtrl.view(iCtrl);
    Property optArm;
    Property optTorso;

    string remoteArmName="/"+robot+"/"+hand;
    optArm.put("device", "remote_controlboard");
    optArm.put("remote",remoteArmName.c_str());
    optArm.put("local",("/"+name+"/localArm/"+hand).c_str());

    string remoteTorsoName="/"+robot+"/torso";
    optTorso.put("device", "remote_controlboard");
    optTorso.put("remote",remoteTorsoName.c_str());
    optTorso.put("local",("/"+name+"/grasplocalTorso/"+hand).c_str());

    robotTorso.open(optTorso);
    robotArm.open(optArm);

    if (!robotTorso.isValid() || !robotArm.isValid())
    {
        fprintf(stdout, "Device not available\n");
        close();
        return false;
    }

    robotArm.view(limArm);
    robotTorso.view(limTorso);

    if (hand=="right_arm")
        arm=new iCubArm("right");
    else
        arm=new iCubArm("left");

    chain=arm->asChain();

    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    deque<IControlLimits*> lim;
    lim.push_back(limTorso);
    lim.push_back(limArm);
    arm->alignJointsBounds(lim);

    arm->setAllConstraints(false);

    thetaMin.resize(10,0.0);
    thetaMax.resize(10,0.0);
    for (unsigned int i=0; i<chain->getDOF(); i++)
    {
        thetaMin[i]=(*chain)(i).getMin();
        thetaMax[i]=(*chain)(i).getMax();
    }

    getAngles(angles, nAngles);
    q.resize(10);
    ones.resize(q.size()); ones=1.0;
    q0.resize(10);
    xdhat.resize(3); 
    odhat.resize(4);

    return true;
}

/************************************************************************/
void OrientationThread::preAskForPose()
{
    iCtrl->storeContext(&currentContext);
    yarp::sig::Vector dof;
    iCtrl->getDOF(dof);
    yarp::sig::Vector newDof;
    newDof=dof;
    newDof[0]=1.0;
    newDof[2]=1.0;
    
    iCtrl->setDOF(newDof,dof);
}

/************************************************************************/
void OrientationThread::postAskForPose()
{
    iCtrl->restoreContext(currentContext);
    iCtrl->deleteContext(currentContext);
}

/************************************************************************/
void OrientationThread::reset()
{
    bestManip=0.0;
    noResult=true;
}

/************************************************************************/
void OrientationThread::setInfo(yarp::sig::Vector &eePos, yarp::sig::Vector &px, yarp::sig::Vector &py, yarp::sig::Vector &pointNormal, yarp::sig::Vector &center, yarp::sig::Vector &biggestAxis)
{
    this->eePos=eePos;
    this->px=px;
    this->py=py;
    this->pointNormal=pointNormal;
    this->center=center;
    this->biggestAxis=biggestAxis;
    done=false;
    work=true;
}

/************************************************************************/
void OrientationThread::getAngles(yarp::sig::Vector &angles, int nAngles)
{
    angles.resize(nAngles);
    double factor=360.0/nAngles;
    double tmp=0.0;
    for (int i=0; i<nAngles; i++)
    {
        angles[i]=tmp;
        tmp+=factor;
    }
}

/************************************************************************/
void OrientationThread::run()
{
    bestOrientation=eye(4,4);
    double manip=0.0;
    Matrix orientation=eye(4,4);

    if (work)
    {
        orientation(0,3)=center[0];
        orientation(1,3)=center[1];
        orientation(2,3)=center[2];
        orientation(0,2)=pointNormal[0];
        orientation(1,2)=pointNormal[1];
        orientation(2,2)=pointNormal[2];

        for (int j=0; j<nAngles; j++)
        {
            orientation(0,0)=px[0]*cos(angles[j])-py[0]*sin(angles[j]);
            orientation(1,0)=px[1]*cos(angles[j])-py[1]*sin(angles[j]);
            orientation(2,0)=px[2]*cos(angles[j])-py[2]*sin(angles[j]);
            orientation(0,1)=px[0]*sin(angles[j])+py[0]*cos(angles[j]);
            orientation(1,1)=px[1]*sin(angles[j])+py[1]*cos(angles[j]);
            orientation(2,1)=px[2]*sin(angles[j])+py[2]*cos(angles[j]);

            yarp::sig::Vector x(3); x[0]=orientation(0,0); x[1]=orientation(1,0); x[2]=orientation(2,0);
            x=x/norm(x);

            if (dot(x,biggestAxis)<-0.4 || dot(x,biggestAxis)>0.4)
                continue;

            if (hand=="right_arm")
            {
                if (orientation(0,0)>0.1 || orientation(2,1)>0.5 || orientation(2,0)<-0.1 || orientation(2,0)>0.9)
                    continue;
            }
            else
            {
                if (orientation(0,0)>0.1 || orientation(2,1)>0.5 || orientation(2,0)<-0.1 || orientation(2,0)>0.9)
                    continue;
            }

            noResult=false;

            od=dcm2axis(orientation);
            q=0.0;       
            manip=0.0;

            iCtrl->askForPose(eePos,od,xdhat,odhat,q);

            q=q*M_PI/180.0;
            arm->setAng(q);

            od[0]=od[0]*od[3];
            od[1]=od[1]*od[3];
            od[2]=od[2]*od[3];
            od.pop_back();

            odhat[0]=odhat[0]*odhat[3];
            odhat[1]=odhat[1]*odhat[3];
            odhat[2]=odhat[2]*odhat[3];
            odhat.pop_back();

            double xdist=norm(eePos-xdhat);
            double odist=norm(od-odhat);

            if (xdist>0.01 && odist>0.1)
                continue;
            
            Jacobian=arm->GeoJacobian();
            mulJac=Jacobian*(Jacobian.transposed());

            manip=sqrt(det(mulJac));

            double limits=0.0;
            for (unsigned int k=0; k<thetaMin.size(); k++)
                limits+=(q[k]-thetaMin[k])*(thetaMax[k]-q[k])/((thetaMax[k]-thetaMin[k])*(thetaMax[k]-thetaMin[k]));

            manip*=(1-exp(-limits));

            if (manip>bestManip)
            {
                bestManip=manip;
                bestOrientation=orientation;
            }
        }
        done=true;
        work=false;
    }
    this->suspend();
}

/************************************************************************/
void OrientationThread::getBestManip(double &manip, yarp::sig::Matrix &orientation)
{
    manip=this->bestManip;
    orientation=this->bestOrientation;
}

/************************************************************************/
bool OrientationThread::checkDone()
{
    return done;
}

/************************************************************************/
bool OrientationThread::getResult()
{
    return noResult;
}

/************************************************************************/
bool OrientationThread::normalDirection(string &hand, yarp::sig::Vector &normal)
{
    if (normal[1]<0 && hand=="left_arm")
        return true;
    if (normal[1]>0 && hand=="right_arm")
        return true;

    return false;
}

/************************************************************************/
void OrientationThread::close() 
{
    if (dCtrl.isValid())
        dCtrl.close();

    if (robotArm.isValid())
        robotArm.close();

    if (robotTorso.isValid())
        robotTorso.close();

    delete arm;
}

/************************************************************************/
void OrientationThread::threadRelease() 
{
    close();
}

