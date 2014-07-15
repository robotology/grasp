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

#ifndef __HELPERS_H_
#define __HELPERS_H_

#include <string>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

class Helpers
{
public:
    static double sign(const double value);
    static void min(const yarp::sig::Matrix &mat, yarp::sig::Vector &out);
    static void max(const yarp::sig::Matrix &mat, yarp::sig::Vector &out);
    static double mod(const double a, const double b);
    static yarp::sig::Vector extractSubVector(const yarp::sig::Vector &vect, const int i, const int j);
};

#endif
