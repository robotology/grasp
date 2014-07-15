#include <string.h>
#include <sstream>
#include "visualizationThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::data3D;

VisualizationThread::VisualizationThread(DataToShow &_data) : data(_data)
{
    running=false;
}

void VisualizationThread::run() 
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int scale=15;
    viewer->initCameraParameters(); 

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

void VisualizationThread::onStop()
{
    running=false;
}
