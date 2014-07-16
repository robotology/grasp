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

#include <string.h>
#include <sstream>
#include "visualizationThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::data3D;

/************************************************************************/
VisualizationThread::VisualizationThread(DataToShow &_data) : data(_data)
{
    running=false;
}

/************************************************************************/
void VisualizationThread::setPosition(int x, int y)
{
    this->x=x;
    this->y=y;
}

/************************************************************************/
void VisualizationThread::setSize(int sizex, int sizey)
{
    this->sizex=sizex;
    this->sizey=sizey;
}

/************************************************************************/
void VisualizationThread::run() 
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int scale=15;
    viewer->initCameraParameters();

    viewer->setPosition(x,y);
    if (sizex!=0 && sizey!=0)
        viewer->setSize(sizex,sizey);

    BoundingBox bb(data.boundingBox.getBoundingBox());
    bb.drawBoundingBox(viewer);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB> (data.cloud)); 
    pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal> (data.normals));
    viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloudPtr,normalPtr,5,0.01,"normals");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudPtr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudPtr, rgb, "rgb");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgb");
    for (int i=0; i<data.sampled_cloud.size(); i++)
    {
        ostringstream temp;
        temp<<i;
        string s=temp.str();
        pcl::PointXYZ point(data.sampled_cloud.at(i).x,data.sampled_cloud.at(i).y, data.sampled_cloud.at(i).z);
        viewer->addSphere (point, 0.002, 0, 1, 0, "pointScored"+s);
    }
    viewer->addCoordinateSystem(0.1);
    viewer->resetCamera();
    running=true;

    while (!viewer->wasStopped())
    {
        if (!running)
        {
            viewer->close();
            break;
        }
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

/************************************************************************/
void VisualizationThread::onStop()
{
    running=false;
}
