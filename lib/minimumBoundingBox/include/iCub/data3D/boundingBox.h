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
 * \defgroup boundingBox boundingBox
 * @ingroup data3D
 *
 * Wrapper to deal with Box3D elements, defined inside objects3D folder. This class provides
 * methods to get and set the main features of a 3D box (corners, orientation, principal axes and their
 * dimension, center). It also provides a function to draw a 3D box in a PCLVisualizer. It needs Point
 * Cloud Library and qhull to be compiled.
 *
 * \author Ilaria Gori
 *
*/

#ifndef __BOUNDING_BOX_H__
#define __BOUNDING_BOX_H__

#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iCub/data3D/Box3D.h>

namespace iCub
{
namespace data3D
{
/**
* @ingroup boundingBox
*
* The Definition of the BoundingBox class.
*/
class BoundingBox
{
    iCub::data3D::Box3D boundingBox;
    yarp::sig::Vector findIndexes(const yarp::sig::Matrix &corner_i);
    yarp::sig::Matrix convertCorners();

    public:

    /**
    * Default Constructor. 
    */
    BoundingBox() {};

    /**
    * Constructor. 
    * @param boundingBox a Box3D object can be set directly in the constructor.
    */
    BoundingBox(const iCub::data3D::Box3D &boundingBox);

    /**
    * Return the Box3D structure underline the bounding box.
    * @return the Box3D.
    */
    iCub::data3D::Box3D getBoundingBox();

    /**
    * Set a Box3D inside the BoundingBox wrapper. 
    * @param boundingBox the set Box3D object.
    */
    void setBoundingBox(const iCub::data3D::Box3D &boundingBox);

    /**
    * Return the corners of the Box3D structure.
    * @return a vector of the 8 corners of the bounding box in 3D.
    */
    std::vector<iCub::data3D::PointXYZ> getCorners();

    /**
    * Set the corners of the bounding box. 
    * @param corners a vector of the 8 corners of the bounding box in 3D.
    */
    void setCorners(const std::vector<iCub::data3D::PointXYZ> &corners);

    /**
    * Return the orientation of the Box3D structure.
    * @return a the orientation matrix of the bounding box.
    */
    yarp::sig::Matrix getOrientation();

    /**
    * Modify the orientation of the bounding box.
    * @param orientation the orientation matrix of the bounding box.
    */
    void setOrientation(const yarp::sig::Matrix &orientation);

    /**
    * Return the dimension of the principal axes, in the same order of
    * the getAxis method.
    * @return a vector containing the dimension of the three principal axes
    * of the bounding box.
    */
    yarp::sig::Vector getDim();

    /**
    * Provide the principal axes of the bounding box.
    * @param x first principal axis of the bounding box.
    * @param y second principal axis of the bounding box.
    * @param z third principal axis of the bounding box.
    */
    void getAxis(yarp::sig::Vector &x, yarp::sig::Vector &y, yarp::sig::Vector &z);

    /**
    * Return the center of the bounding box.
    * @return a vector representing the center of the bounding box.
    */
    yarp::sig::Vector getCenter();

    /**
    * Draw a 3D box in a PCLVisualizer window.
    * @param viewer the PCLVisualizer window where the 3D box has to be drawn.
    * @param viewport the viewport inside the PCLVisualizer where the 3D box has to be drawn.
    */
    void drawBoundingBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int viewport=0);
};
}
}
#endif

