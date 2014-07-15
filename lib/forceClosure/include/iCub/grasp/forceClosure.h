#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <deque>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <iCub/ctrl/math.h>

#ifndef __FORCECLOSURE_H__
#define __FORCECLOSURE_H__

namespace iCub
{
namespace grasp
{
struct ContactPoints
{
    yarp::sig::Vector c1;
    yarp::sig::Vector c2;
    yarp::sig::Vector c3;
    yarp::sig::Vector n1;
    yarp::sig::Vector n2;
    yarp::sig::Vector n3;
    //angle between cone normal and cone lines
    double alpha1;
    double alpha2;
    double alpha3;
    int ov_cones;
};

struct ConeBounds
{
    yarp::sig::Vector n1;
    yarp::sig::Vector n2;
};

    bool isForceClosure(const ContactPoints &contactPoints);

}
}

#endif

