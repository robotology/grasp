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

#ifndef __PRECISION_GRASP_H__
#define __PRECISION_GRASP_H__

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <boost/thread/thread.hpp>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/grasp/forceClosure.h>
#include <visualizationThread.h>
#include <psoThread.h>

#ifdef _WIN32
    #include "custom/dirent.h"
#endif

#define STATE_WAIT      0
#define STATE_ESTIMATE  1
#define STATE_IK        3
#define STATE_GRASP     2
#define RIGHT_HAND      "right"
#define LEFT_HAND       "left"
#define NO_HAND         "no_hand"

class PrecisionGrasp: public yarp::os::RFModule
{
private:
    int current_state;
    int nFile;
    int counter;
    int winner_triplet;
    int context_in_right;
    int current_context_right;
    int context_right;
    int context_in_left;
    int current_context_left;
    int context_left;
    int ov_cones1;
    int ov_cones2;
    int ov_cones3;
    int ov_cones4;
    int winner_ov_cones;
    int posx;
    int posy;
    int sizex;
    int sizey;
    bool clicked;
    bool fromFile;
    bool fromFileFinished;
    bool dont;
    bool grasped;
    bool visualize;
    bool straight;
    bool readyToGrasp;
    bool rightBlocked;
    bool leftBlocked;
    bool filterCloud;
    bool writeCloud;
    bool rightDisabled;
    bool leftDisabled;
    double handSize;
    double radiusSearch;
    double sampling;
    double alpha;
    double bestCost;
    double bestManipulability;
    std::string path;
    std::string filenameTrain;
    std::string robot;
    std::string outputFile;
    ofstream graspFileTrain;
    std::string winner_hand;

    std::vector<yarp::sig::Vector> contacts_r1;
    std::vector<yarp::sig::Vector> normals_r1;
    std::vector<yarp::sig::Vector> contacts_r2;
    std::vector<yarp::sig::Vector> normals_r2;
    std::vector<yarp::sig::Vector> contacts_r3;
    std::vector<yarp::sig::Vector> normals_r3;
    std::vector<yarp::sig::Vector> contacts_r4;
    std::vector<yarp::sig::Vector> normals_r4;
    yarp::sig::Vector joints_tmp;
    yarp::sig::Vector winner_joints;
    yarp::sig::Vector ee_tmp;
    yarp::sig::Vector winner_ee;
    yarp::sig::Vector axis_angle_tmp;
    yarp::sig::Vector winner_axis;
    yarp::sig::Vector combination_tmp;
    yarp::sig::Vector winner_combination;
    yarp::sig::Vector winner_odhat;
    yarp::sig::Vector winner_xdhat;
    yarp::sig::Vector center;
    yarp::sig::Vector dim;
    yarp::sig::Matrix rotation;
    yarp::sig::Vector xdhat,odhat;
    yarp::sig::Vector q;
    yarp::sig::Vector offsetR;
    yarp::sig::Vector offsetL;
    yarp::sig::Vector home_p_l;
    yarp::sig::Vector home_p_r;
    yarp::sig::Vector home_o_l;
    yarp::sig::Vector home_o_r;

    yarp::dev::PolyDriver dCtrlRight, dCtrlLeft;
    yarp::dev::ICartesianControl *iCtrlRight, *iCtrlLeft;
    yarp::dev::IPositionControl *posRight, *posLeft;
    yarp::dev::IEncoders *encRight, *encLeft;
    yarp::dev::PolyDriver robotArmRight, robotArmLeft;
    yarp::dev::PolyDriver robotTorso;
    yarp::dev::IControlLimits *limTorso, *limArmRight, *limArmLeft;

    iCub::iKin::iCubArm *armRight, *armLeft;
    iCub::iKin::iKinChain *chainRight, *chainLeft;

    yarp::sig::Vector thetaMinRight, thetaMinLeft;
    yarp::sig::Vector thetaMaxRight, thetaMaxLeft;

    VisualizationThread *visualizationThread;
    PsoThread* psoThreadFitness1;
    PsoThread* psoThreadFitness2;
    PsoThread* psoThreadFitness3;
    PsoThread* psoThreadFitness4;
    DataToShow data;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;
    pcl::PointCloud <pcl::Normal>::Ptr normals;

    yarp::os::Port reconstructionPort;
    yarp::os::Port ikPort1r;
    yarp::os::Port ikPort2r;
    yarp::os::Port ikPort3r;
    yarp::os::Port ikPort4r;
    yarp::os::Port ikPort1l;
    yarp::os::Port ikPort2l;
    yarp::os::Port ikPort3l;
    yarp::os::Port ikPort4l;
    yarp::os::Port areCmdPort;
    yarp::os::Port depth2kin;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> meshPort;
    yarp::os::Port rpc;
    yarp::os::BufferedPort<yarp::os::Bottle> toMatlab;

    yarp::os::Semaphore mutex;
    yarp::os::Semaphore mutex_to_write;
    yarp::os::Event eventRpc;

    iCub::data3D::BoundingBox boundingBox;

    double min(const double a, const double b) {return (a>b)?b:a;};
    double max(const double a, const double b) {return (a<b)?b:a;};

    void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void addPlanePoints();
    void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second=false);
    void write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const std::string &fileName);
    bool normalPointingOut(pcl::Normal &normal, pcl::PointXYZ &point);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    void fromSurfaceMesh(const iCub::data3D::SurfaceMeshWithBoundingBox& msg);
    bool fillCloudFromFile();
    void askToGrasp();
    yarp::os::Bottle prepareData(const iCub::grasp::ContactPoints &triplet, const int c);
    std::string extractData(const yarp::os::Bottle &data, const int t);
    void fillVectorFromBottle(const yarp::os::Bottle* b, yarp::sig::Vector &v);
    double getZDim(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz);
    void readData(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud <pcl::Normal>::Ptr n);
    void sampleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud <pcl::Normal>::Ptr n);
    void writeIKresult(const std::vector<yarp::sig::Vector> &contacts_r, const std::vector<yarp::sig::Vector> &normals_r, const int c, const std::string &hand, const int ov_cones);
    void writeIKBestresult();
    void write(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud <pcl::Normal>::Ptr n, const std::string &fileName);
    void associateDim();
    bool openDevices();
    void writeToMatlab(const std::vector<yarp::sig::Vector> &contacts_r, const std::vector<yarp::sig::Vector> &normals_r, const std::string &hand);
    void writeBestSolution();
    void fillBottleFromVector(const yarp::sig::Vector &vect, yarp::os::Bottle *b);

public:

    PrecisionGrasp();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
    double getPeriod();
};

#endif

