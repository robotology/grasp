/*
 * VISUALIZER THREAD for 3D OBJECT DISPLAY
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email: tanis.mar@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __PCLVISTHRD_H__
#define __PCLVISTHRD_H__

// Includes

#include <iostream>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>


class VisThread: public yarp::os::RateThread
{
protected:
    // EXTERNAL VARIABLES: set on call
    std::string id;

    // INTERNAL VARIABLES:
    // Mutex cotnrol
    bool update;
    boost::mutex updateModelMutex;

    // Visualizer global variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Flow control flags
    bool initialized;
    bool addClouds;
    bool clearing;
    bool updatingCloud;
    bool displayNormals;
    bool displayBB;

    // Processing parameters
    bool minimumBB;
    double radiusSearch;

    /**
     * @brief updateVis -  Unlocks the mutex on a cycle so that the visualizer and its contents can be updated
     */
    void updateVis();

    /**
     * @brief plotBB - Computes and displays Bounding Box
     * @param minBB - true for oriented (minimum) boinding box. False for axis aligned bounding box
     */
    void plotBB(bool minBB);            // Function called within the update loop.

    /**
     * @brief plotNormals - Computes and displays Normals
     * @param rS - radiusSearch for consider nerighbors to compute surface normal.
     */
    void plotNormals(double rS);        // Function called within the update loop.

public:
    // CONSTRUCTOR
    VisThread(int period, const std::string &_cloudname);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
     * @brief addNormals - Sets flow to allow bounding box to be computed and added on display update.
     * @param rS - radiusSearch for consider nerighbors to compute surface normal.
     */
    void addNormals(double rS);         // Function called from main module to set parameters and unlock update

    /**
     * @brief addBoundingBox
     * @param minBB - true for oriented (minimum) boinding box. False for axis aligned bounding box
     */
    void addBoundingBox(bool minBB);    // Function called from main module to set parameters and unlock update

    /**
     * @brief clearVisualizer - Clears all the figures (except the coordinate system) from the display.
     */
    void clearVisualizer();

    /**
     * @brief updateCloud - Updates the displayed cloud with the received one.
     * @param cloud_in - Input cloud to be displayed
     */
    void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

    /**
     * @brief accumulateClouds - Selects between displaying clouds together or not
     * @param accum - True to accumulate clouds on display. False to display them independently
     */
    void accumulateClouds(bool accum);


};

#endif


