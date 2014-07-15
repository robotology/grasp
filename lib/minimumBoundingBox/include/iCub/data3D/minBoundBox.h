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
 * \defgroup minimumBoundingBox minimumBoundingBox
 * @ingroup data3D
 *
 * Static function to compute the minimum enclosing bounding box given a set of points in 3D.
 * It needs Point Cloud Library to be compiled.
 *
 * \author Ilaria Gori
 *
*/

#ifndef __MIN_BOUND_BOX_H__
#define __MIN_BOUND_BOX_H__

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
#include <iCub/data3D/boundingBox.h>

namespace iCub
{
namespace data3D
{
namespace MinimumBoundingBox
{
    /**
    * Given a point cloud (defined in the Point Cloud Library), it computes the minimum enclosing
    * bounding box.
    * @param cloud the input point cloud.
    * @return a BoundingBox object, which encloses the input point cloud.
    **/
    iCub::data3D::BoundingBox getMinimumBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}
}
}
#endif


