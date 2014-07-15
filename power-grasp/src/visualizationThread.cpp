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
    x=0;
    y=0;
    sizex=0;
    sizey=0;
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
    viewer->setBackgroundColor (0, 0, 0);

    viewer->setPosition(x,y);
    if (sizex!=0 && sizey!=0)
        viewer->setSize(sizex,sizey);

    for (unsigned int i=0; i<data.goodPointsIndexes->size(); i++)
    {
        ostringstream temp;
        temp<<i;
        string s=temp.str();
        pcl::PointXYZ point(data.cloud->points.at((*data.goodPointsIndexes)[i]).x, data.cloud->points.at((*data.goodPointsIndexes)[i]).y, data.cloud->points.at((*data.goodPointsIndexes)[i]).z);
        viewer->addSphere (point, 0.002, 1, 1, 0, "pointScored"+s);
    }

    pcl::PointXYZ origin(data.chosenPoint[0],data.chosenPoint[1],data.chosenPoint[2]);
    pcl::PointXYZ normalOrigin;
    pcl::PointXYZ normalOriginScaled;
    if (data.hand=="left")
    {
        normalOrigin.x=origin.x+data.chosenOrientation(0,2);
        normalOrigin.y=origin.y+data.chosenOrientation(1,2);
        normalOrigin.z=origin.z+data.chosenOrientation(2,2);
        normalOriginScaled.x=origin.x+(data.chosenOrientation(0,2)/scale);
        normalOriginScaled.y=origin.y+(data.chosenOrientation(1,2)/scale);
        normalOriginScaled.z=origin.z+(data.chosenOrientation(2,2)/scale);
    }
    else
    {
        normalOrigin.x=origin.x-data.chosenOrientation(0,2);
        normalOrigin.y=origin.y-data.chosenOrientation(1,2);
        normalOrigin.z=origin.z-data.chosenOrientation(2,2);
        normalOriginScaled.x=origin.x-(data.chosenOrientation(0,2)/scale);
        normalOriginScaled.y=origin.y-(data.chosenOrientation(1,2)/scale);
        normalOriginScaled.z=origin.z-(data.chosenOrientation(2,2)/scale);
    }

    viewer->addSphere (origin, 0.002, 1, 0, 0, "pointChosen");
    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(origin,normalOriginScaled,0,0,1,0,"pz");

    pcl::PointXYZ xaxis(normalOrigin.x+data.chosenOrientation(0,0),normalOrigin.y+data.chosenOrientation(1,0),normalOrigin.z+data.chosenOrientation(2,0));
    pcl::PointXYZ xaxisScaled(normalOriginScaled.x+(data.chosenOrientation(0,0)/scale),normalOriginScaled.y+(data.chosenOrientation(1,0)/scale),normalOriginScaled.z+(data.chosenOrientation(2,0)/scale));

    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(xaxisScaled,normalOriginScaled,1,0,0,0,"px");

    pcl::PointXYZ yaxis(normalOrigin.x+data.chosenOrientation(0,1),normalOrigin.y+data.chosenOrientation(1,1),normalOrigin.z+data.chosenOrientation(2,1));
    pcl::PointXYZ yaxisScaled(normalOriginScaled.x+(data.chosenOrientation(0,1)/scale),normalOriginScaled.y+(data.chosenOrientation(1,1)/scale),normalOriginScaled.z+(data.chosenOrientation(2,1)/scale));

    viewer->addArrow<pcl::PointXYZ,pcl::PointXYZ>(yaxisScaled,normalOriginScaled,0,1,0,0,"py");

    BoundingBox bb(data.boundingBox->getBoundingBox());
    bb.drawBoundingBox(viewer);

    viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(data.cloud,data.normals,10,0.01,"normals");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(data.cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (data.cloud, rgb, "rgb");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgb");

    viewer->resetCamera();
    viewer->setCameraPosition(-0.0611749,-0.040113,0.00667606,-0.105521,0.0891437,0.990413);

    mutex.wait();
    running=true;
    mutex.post();

    while (!viewer->wasStopped())
    {
        mutex.wait();
        if (!running)
        {
            viewer->close();
            mutex.post();
            break;
        }
        mutex.post();
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

/************************************************************************/
void VisualizationThread::onStop()
{
    mutex.wait();
    running=false;
    mutex.post();
}
