// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  King's College London
  * Author: Kris De Meyer, Giuseppe Cotugno
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

/**
 * @file IcubVisThread.h
 * @brief Visualisation thread for the Icub
 * \author Kris De Meyer
 * \author Giuseppe Cotugno
 */

#pragma once

//yarp includes
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

//local includes
#include "PortMap.h"



namespace darwin {
namespace grasp {


class IcubVisThread : public yarp::os::Thread {
public:
    /**
    * default constructor
    */
    IcubVisThread();

    /**
     * destructor
     */
    ~IcubVisThread();

	/**
    *  configure the ports and parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    virtual bool configure(yarp::os::ResourceFinder& rf);

    /**
    * function that initialise the thread
    */
    virtual bool threadInit();

    /**
    * function called when the thread is stopped
    */
    virtual void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    
     
    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);
private:   

    PortMap<yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > > _visPorts;
};

} //end namespace grasp
} //end namespace darwin

//----- end-of-file --- ( next line intentionally left blank ) ------------------
