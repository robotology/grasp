#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>
#include "handIK.h"

#ifndef __HANDIKMODULE_H__
#define __HANDIKMODULE_H__

class HandIKModule: public yarp::os::RFModule
{
    int winnerIndex;
    double bestObjValue;
    bool done;
    bool work;
    std::string hand;
    std::string portName;
    std::string tag;
    yarp::sig::Vector center;
    yarp::sig::Vector dim;
    yarp::sig::Matrix rotation;
    yarp::sig::Matrix rot_tran;
    std::vector<yarp::sig::Vector> contacts_r;
    std::vector<yarp::sig::Vector> normals_r;
    std::vector<yarp::sig::Vector> contacts_o;
    std::vector<yarp::sig::Vector> normals_o;

    std::vector<yarp::sig::Vector> combinations;

    HandIK_Variables bestSolution;

    yarp::os::Port outputPort;
    yarp::os::Port rpc;

private:
    void fromRootToObject();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    void createCombinationVector();
    bool extractData(const yarp::os::Bottle &data);
    void fillVectorFromBottle(const yarp::os::Bottle* b, yarp::sig::Vector &v);
    void fillMatrixFromBottle(const yarp::os::Bottle* b, yarp::sig::Matrix &m, int rows, int cols);
    void prepareData(yarp::os::Bottle &data);
    double evaluateFingers(const HandIK_Variables &solution, const int id);

public:

    HandIKModule();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
    double getPeriod();
};

#endif

