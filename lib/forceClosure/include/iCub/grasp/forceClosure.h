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
 * \defgroup forceClosure forceClosure
 * @ingroup grasp
 *
 * Static function that tells you if a set of contact points is force closure or not.
 *
 * \author Ilaria Gori
 *
*/

#ifndef __FORCECLOSURE_H__
#define __FORCECLOSURE_H__

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

namespace iCub
{
namespace grasp
{
/**
* @ingroup forceClosure
*
* Definition of the ForceClosure.
*/

/**
* struct that represents a triplet of contact points along with their normals and the friction coefficient
* of the friction cones. The ov_cones parameter is used by the precision-grasp module.
**/
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

    /**
    * Given a set of ContactPoints, it computes the force closure property.
    * @param contactPoints a set of ContactPoints as defined in the above-mentioned
    * structure.
    * @return true if the triplet is force closure, false otherwise.
    **/
    bool isForceClosure(const ContactPoints &contactPoints);

}
}

#endif

