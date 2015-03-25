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

#include <reconstructionRoutine.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::ctrl;

/************************************************************************/
ReconstructionRoutine::ReconstructionRoutine() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), cloudComplete(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    resetClouds();
}

/************************************************************************/
bool ReconstructionRoutine::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    string configFileDisparity=opt.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str();
    string cameraContext=opt.check("CameraContext",Value("cameraCalibration")).asString().c_str();
    string name=opt.check("name",Value("object-reconstruction")).asString().c_str();
    disparityOut.open("/"+name+"/depth:o");

    ResourceFinder cameraFinder;
    cameraFinder.setDefaultContext(cameraContext.c_str());
    cameraFinder.setDefaultConfigFile(configFileDisparity.c_str());
    cameraFinder.setVerbose();
    cameraFinder.configure(0,NULL);

    disp=new DisparityThread(name,cameraFinder, false, false, true); 
    disp->start();

    return true;
} 

/************************************************************************/
void ReconstructionRoutine::close()
{
    disp->stop();
    delete disp;
}

/************************************************************************/
void ReconstructionRoutine::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxfilter;
    voxfilter.setInputCloud (cloud_in);
    voxfilter.setLeafSize (0.01f, 0.01f, 0.01f);
    voxfilter.filter (*cloud_downsampled);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_downsampled);
    sor.setMeanK (cloud_downsampled->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

/************************************************************************/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReconstructionRoutine::getPointCloud()
{
    return this->cloud;
}

/************************************************************************/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReconstructionRoutine::getPointCloudComplete()
{
    return this->cloudComplete;
}

/************************************************************************/
void ReconstructionRoutine::resetClouds()
{
    cloud->clear();
    cloudComplete->clear();
}

/************************************************************************/
bool ReconstructionRoutine::triangulateSinglePoint(IplImage* imgL, IplImage* imgR, yarp::sig::Vector &point2D, yarp::sig::Vector &point3D)
{
    point3D.resize(3,0.0);
    Mat leftIm(imgL);
    Mat rightIm(imgR);
    disp->setImages(leftIm,rightIm);
    while(!disp->checkDone())
        yarp::os::Time::delay(0.1);

    Point2f pixel(point2D[0],point2D[1]);

    Point3f point;
    disp->triangulate(pixel,point);
    if (point.z==0)
        return false;

    point3D[0]=point.x;
    point3D[1]=point.y;
    point3D[2]=point.z;

    return true;
}

/************************************************************************/
bool ReconstructionRoutine::reconstruct(IplImage* imgL, IplImage* imgR, Bottle& pixelList)
{
    Mat leftIm(imgL);
    Mat rightIm(imgR);
    disp->setImages(leftIm,rightIm);
    while(!disp->checkDone())
        yarp::os::Time::delay(0.1);

    triangulation(imgL,cloudComplete,pixelList);
    filter(cloudComplete,cloud);

    return true;
}

/************************************************************************/
bool ReconstructionRoutine::updateDisparity(IplImage* imgL, IplImage* imgR)
{
    Mat leftIm(imgL);
    Mat rightIm(imgR);
    disp->setImages(leftIm,rightIm);
    while(!disp->checkDone())
        yarp::os::Time::delay(0.1);

    Mat disparity;
    disp->getDisparity(disparity);
    IplImage disparityImg=disparity;
    ImageOf<PixelMono> depthToDisplay;
    depthToDisplay.wrapIplImage(&disparityImg);
    disparityOut.prepare()=depthToDisplay;
    disparityOut.write();

    return true;
}

/************************************************************************/
void ReconstructionRoutine::triangulation(IplImage* imgL, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, yarp::os::Bottle& pixelList)
{
    Bottle* listOfPoints=pixelList.get(0).asList();
	int n=0;
    for (int i=0; i<listOfPoints->size(); i++)
    {
        Point2f point2D;
        Bottle* point=listOfPoints->get(i).asList();
        point2D.x=point->get(0).asDouble();
        point2D.y=point->get(1).asDouble();

        Point3f point3D;
        disp->triangulate(point2D,point3D);
        if (point3D.z==0)
            continue;

        CvScalar color=cvGet2D(imgL,point2D.y,point2D.x);

        pcl::PointXYZRGB newPoint;
        newPoint.x=point3D.x;
        newPoint.y=point3D.y;
        newPoint.z=point3D.z;

        newPoint.r=color.val[0];
        newPoint.g=color.val[1];
        newPoint.b=color.val[2];

        uint8_t r=color.val[0];
        uint8_t g=color.val[1];
        uint8_t b=color.val[2];
 
        newPoint.rgba = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        input->push_back(newPoint);
    }
}

