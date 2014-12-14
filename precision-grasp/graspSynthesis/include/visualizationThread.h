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

#ifndef __VISUALIZATION_THREAD_H__
#define __VISUALIZATION_THREAD_H__

#include <yarp/os/Thread.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <iCub/data3D/minBoundBox.h>
#include <iCub/data3D/boundingBox.h>

struct DataToShow
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::Normal> normals;
    pcl::PointCloud<pcl::PointXYZ> sampled_cloud;
    iCub::data3D::BoundingBox boundingBox;
    std::string hand;
};

class VisualizationThread : public yarp::os::Thread
{
private:
    DataToShow &data;
    bool running;
    int x;
    int y;
    int sizex;
    int sizey;

public:

    VisualizationThread(DataToShow &_data);
    ~VisualizationThread() {};

    void onStop();
    void run();
    void setPosition(int x,int y);
    void setSize(int sizex, int sizey);
};

#endif
