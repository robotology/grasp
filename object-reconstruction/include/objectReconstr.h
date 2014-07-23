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

#ifndef __OBJECT_RECONSTRUCTION_MODULE_H__
#define __OBJECT_RECONSTRUCTION_MODULE_H__

#include <fstream>
#include <yarp/os/Vocab.h>
#include <yarp/os/RFModule.h>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/data3D/minBoundBox.h>
#include <reconstructionRoutine.h>

#define ACK                     VOCAB3('a','c','k')
#define NACK                    VOCAB4('n','a','c','k')

#define STATE_WAIT              0
#define STATE_RECONSTRUCT       1
#define STATE_VISUALIZE         2

class ObjectReconstr: public yarp::os::RFModule
{
    int currentState;
    int number;
    double middlex;
    double middley;
    bool write;
    bool closing;
    bool visualizationOn;
    bool computeBB;
    std::string outputDir;

    iCub::data3D::BoundingBox boundingBox;

    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortInRight;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> pointCloudPort;
    yarp::os::RpcClient segmentationPort;

    ReconstructionRoutine recRoutine;

    void visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    bool updateCloud();
	void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second=false);
    yarp::os::Bottle getPixelList();
    void savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

public:

    ObjectReconstr();
    bool configure(ResourceFinder &rf);
    bool close();
    bool updateModule();
    bool interruptModule();
    double getPeriod();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

};

#endif
