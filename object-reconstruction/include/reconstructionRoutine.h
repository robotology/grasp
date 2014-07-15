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

#ifndef __RECONSTRUCTION_ROUTINE_H__
#define __RECONSTRUCTION_ROUTINE_H__

#include <sstream>
#include <cv.h>
#include <yarp/sig/Image.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <iCub/stereoVision/disparityThread.h>
#include <iCub/segmentation/SegmentationModuleInterface.h>

class ReconstructionRoutine
{
private:

    DisparityThread* disp;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > disparityOut;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudComplete;

    void triangulation(IplImage* imgL, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, yarp::os::Bottle& pixelList);
    void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered);

public:

    ReconstructionRoutine();
    ~ReconstructionRoutine() {};

    void resetClouds();
    bool reconstruct(IplImage* imgL, IplImage* imgR, yarp::os::Bottle& pixelList);
    bool updateDisparity(IplImage* imgL, IplImage* imgR);
    bool triangulateSinglePoint(IplImage* imgL, IplImage* imgR, yarp::sig::Vector &point2D, yarp::sig::Vector &point3D);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudComplete();

    void close();
    bool open(const yarp::os::Property &options);
};

#endif
