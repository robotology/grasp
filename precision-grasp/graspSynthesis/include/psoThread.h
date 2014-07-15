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

#ifndef __PSO_THREAD_H__
#define __PSO_THREAD_H__

#include <cmath>
#include <algorithm>
#include <fstream>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/grasp/forceClosure.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>

class PsoThread : public yarp::os::RateThread
{
private:
    bool done;
    bool work;
    bool init;
    bool set;
    int particles;
    int iterations;
    int overlapping_cones;
    int dimension;
    int rand_method;
    double omega;
    double phi_p;
    double phi_g;
    double limit_finger_min;
    double limit_finger_max;
    double hand_area;
    double fg;
    double cost;
    double alpha;
    double vmax,vmin;

    std::deque<yarp::sig::Vector> x;
    std::deque<yarp::sig::Vector> ns;
    std::deque<yarp::sig::Vector> v;
    std::deque<yarp::sig::Vector> p;
    yarp::sig::Vector nsg;
    yarp::sig::Vector fp;
    yarp::sig::Vector g;
    std::vector<yarp::sig::Vector> visited;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud <pcl::Normal>::Ptr normals;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    yarp::sig::Vector findDifferentIds(const yarp::sig::Vector &ids);
    yarp::sig::Vector findTriplet(const yarp::sig::Vector &vect);
    int findClosestPoint(pcl::PointXYZ &point);
    yarp::sig::Vector assignPoints(const yarp::sig::Vector &ids);
    yarp::sig::Vector assignNormals(const yarp::sig::Vector &ids);
    double fitness(iCub::grasp::ContactPoints &triplet);
    bool counterOverlap(yarp::sig::Vector &n1, yarp::sig::Vector &n2, double &angle);
    double penaltyCones(int n_cones, yarp::sig::Vector &idx, yarp::sig::Vector &angles);
    double triangleArea(yarp::sig::Vector &p1, yarp::sig::Vector &p2, yarp::sig::Vector &p3);
    iCub::grasp::ContactPoints fillContactPoints(yarp::sig::Vector &xi, yarp::sig::Vector &nsi);
    void initialization();
    yarp::sig::Vector findRandomId();

public:

    PsoThread();
    ~PsoThread() {};

    bool open(const yarp::os::Property &options);
    bool checkDone();
    void threadRelease();
    void close();
    void run();
    void setData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::Normal>::Ptr normals, double alpha, int overlapping_cones);
    iCub::grasp::ContactPoints getBestTriplet();
    double getCost();
    bool isSuccessful();
 
};

#endif
