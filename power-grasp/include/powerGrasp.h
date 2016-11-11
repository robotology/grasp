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

#ifndef __POWER_GRASP_MODULE_H__
#define __POWER_GRASP_MODULE_H__

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>

#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <yarp/os/all.h>

#include <iCub/learningMachine/FixedRangeScaler.h>
#include <iCub/learningMachine/LSSVMLearner.h>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>

#include "visualizationThread.h"
#include "orientationThread.h"

#ifdef _WIN32
    #include "custom/dirent.h"
#endif

#define similarity      0.005
#define STATE_WAIT      0
#define STATE_ESTIMATE  1
#define STATE_GRASP     2
#define RIGHT_HAND      "right"
#define LEFT_HAND       "left"
#define NO_HAND         "no_hand"
#define ACK             "ack"
#define NACK            "nack"
#define MODALITY_RIGHT  0
#define MODALITY_LEFT   1
#define MODALITY_TOP    2
#define MODALITY_CENTER 3
#define MODALITY_AUTO   4

class PowerGrasp: public yarp::os::RFModule
{
private:
    int currentState;
    int currentModality;
    int numberOfBestPoints;
    int winnerIndex;
    int modality;
    int nFile;
    int posx;
    int posy;
    int sizex;
    int sizey;
    bool fromFile;
    bool fromFileFinished;
    bool grasped;
    bool visualize;
    bool straight;
    bool train;
    bool testWithLearning;
    bool readyToGrasp;
    bool rightBlocked;
    bool leftBlocked;
    bool rightDisabled;
    bool leftDisabled;
    bool filterCloud;
    bool graspSpecificPoint;
    bool blockRightTmp;
    bool blockLeftTmp;
    bool testWithLearningEnabled;
    bool writeCloud;
    bool noResult;
    bool tooFar;
    double handSize;
    double fingerSize;
    double radiusSearch;
    double maxCurvature;
    double maxy;
    double maxz;
    double dimy;
    double dimz;
    float bestCurvature;
    float currentCurvature;
    std::string chosenHand;
    std::string path;
    std::string outputDir;

    VisualizationThread *visualizationThread;
    OrientationThread* orientationThreadRight;
    OrientationThread* orientationThreadLeft;
    DataToShow data;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;
    pcl::PointCloud <pcl::Normal>::Ptr normals;

    yarp::sig::Vector offsetR;
    yarp::sig::Vector offsetL;
    yarp::sig::Vector chosenPoint;
    yarp::sig::Vector chosenNormal;
    yarp::sig::Vector chosenPixel;
    yarp::sig::Matrix chosenOrientation;
    
    yarp::os::Port reconstructionPort;
    yarp::os::Port areCmdPort;
    yarp::os::Port depth2kin;
    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> meshPort;
    yarp::os::Port rpc;

    yarp::os::Mutex mutex;
    yarp::os::Event eventRpc;

    std::vector<double> rankScores;
    std::vector<int> rankIndices;

    iCub::data3D::BoundingBox boundingBox;

    iCub::learningmachine::FixedRangeScaler scalerIn,scalerOut;
    iCub::learningmachine::LSSVMLearner     machine;

    double min(const double a, const double b) {return (a>b)?b:a;};
    double max(const double a, const double b) {return (a<b)?b:a;};

    void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void addPlanePoints();
    void configureSVM(yarp::os::Bottle &bottleSVM);
    void configureGeneralInfo(yarp::os::ResourceFinder &rf);
    void normalEstimation();
    void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered);
    void write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const int nFile);
    void insertElement(const double score, const int index);
    void fromSurfaceMesh(const iCub::data3D::SurfaceMeshWithBoundingBox& msg);
    void askToGrasp();
    void resetBools();
    void manageModality();
    void chooseCandidatePoints();
    void rankPoints();
    void startVisualization();
    void computeDim();
    int findIndexFromCloud(const yarp::sig::Vector &point);
    double scoreFunction(const int index);
    bool normalPointingOut(const int index, const yarp::sig::Vector &center);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool fillCloudFromFile();
    bool get3DPoint(const yarp::sig::Vector &point2D, yarp::sig::Vector &point3D);
    std::string chooseBestPointAndOrientation(int &winnerIndex, yarp::sig::Matrix &designedOrientation);
    yarp::sig::Vector vectorFromNormal(const int index);
    yarp::sig::Vector vectorFromCloud(const int index);
    yarp::sig::Vector pointFromBottle(const yarp::os::Bottle &bot, const int index);
    yarp::sig::Vector assignIndexToAxes(double &anglez);
    yarp::sig::Vector findBiggestAxis(int &ind);
    yarp::sig::Vector computeApproachVector(const yarp::sig::Vector &chosenPoint);
    std::vector<pcl::PointIndices> selectBigClusters(const std::vector<pcl::PointIndices> &clusters);

public:

    PowerGrasp();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
    double getPeriod();
};

#endif

