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

#include <objectReconstr.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::data3D;

/************************************************************************/
ObjectReconstr::ObjectReconstr()
{
    currentState=STATE_WAIT;
    write=false;
    visualizationOn=false;
    closing=false;
    number=0;
}

/************************************************************************/
bool ObjectReconstr::configure(ResourceFinder &rf)
{
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string name=rf.check("name",Value("object-reconstruction")).asString().c_str();
    setName(name.c_str());
    outputDir=rf.getHomeContextPath().c_str();
    computeBB=rf.check("computeBB",Value(false)).asBool();

    middlex=-1;
    middley=-1;

    fileName = "3Dobject";

    string slash="/";

    string imL=slash + getName().c_str() + "/left:i";
    string imR=slash + getName().c_str() + "/right:i";
    imagePortInLeft.open(imL.c_str());
    imagePortInRight.open(imR.c_str());

    bool ok=true;
    if (robot=="icub")
    {
        ok&=Network::connect((slash+robot+"/camcalib/left/out").c_str(),imL.c_str());
        ok&=Network::connect((slash+robot+"/camcalib/right/out").c_str(),imR.c_str());
    }
    else
    {
        ok&=Network::connect((slash+robot+"/cam/left").c_str(),imL.c_str());
        ok&=Network::connect((slash+robot+"/cam/right").c_str(),imR.c_str());
    }

    if (!ok)
    {
        printf("Cameras not available, closing\n");
        imagePortInLeft.close();
        imagePortInRight.close();
        return false;
    }
    segmentationPort.open((slash+getName().c_str()+"/segmentation").c_str());

    pointCloudPort.open((slash + getName().c_str() + "/mesh:o").c_str());
    rpc.open((slash + getName().c_str() + "/rpc").c_str());
    attach(rpc);

    Property recRoutOptions;
    recRoutOptions.put("ConfigDisparity",rf.check("ConfigDisparity",Value("icubEyes.ini")).asString().c_str());
    recRoutOptions.put("CameraContext",rf.check("CameraContext",Value("cameraCalibration")).asString().c_str());
    recRoutOptions.put("name",getName().c_str());

    if (!recRoutine.open(recRoutOptions))
    {
        fprintf(stdout, "Problem with thread, the module will be closed\n");
        close();
        return false;
    }

    // Visualizer Thread
    visThrd = new VisThread(50, "Cloud");
    if (!visThrd->start())
    {
        delete visThrd;
        visThrd = 0;
        cout << "\nERROR!!! visThread wasn't instantiated!!\n";
        return false;
    }
    cout << "PCL visualizer Thread istantiated...\n";

    return true;
}

/************************************************************************/
bool ObjectReconstr::close()
{
    imagePortInLeft.close();
    imagePortInRight.close();

    rpc.close();
    pointCloudPort.close();
    segmentationPort.close();

    recRoutine.close();
    if (visThrd)    //Close visualization thread clean.
    {
        visThrd->stop();
        delete visThrd;
        visThrd =  0;
    }

    return true;
}

/************************************************************************/
Bottle ObjectReconstr::getPixelList()
{
    Bottle toSend, reply;
    toSend.addString("get_component_around");
    toSend.addInt((int)middlex);
    toSend.addInt((int)middley);

    segmentationPort.write(toSend,reply);
    return reply;
}

/************************************************************************/
void ObjectReconstr::savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   const string& name)
{
    stringstream s;
    s.str("");
    s<<outputDir + "/" + name.c_str() <<number;
    string filename=s.str();
    string filenameNumb=filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++)
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

    plyfile.close();

    number++;
    fprintf(stdout, "Writing finished\n");
}

/************************************************************************/
bool ObjectReconstr::updateCloud()
{
    ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(true);
    ImageOf<PixelRgb> *tmpR = imagePortInRight.read(true);

    IplImage* imgL;
    IplImage* imgR;
    if(tmpL!=NULL && tmpR!=NULL)
    {
        imgL= (IplImage*) tmpL->getIplImage();
        imgR= (IplImage*) tmpR->getIplImage();
    }
    else
    {
        fprintf(stdout, "Problem with image ports occurred\n");
        return false;
    }

    if (currentState==STATE_RECONSTRUCT)
    {
        Bottle pixelList=getPixelList();
        return recRoutine.reconstruct(imgL,imgR,pixelList);
    }
    else
        return recRoutine.updateDisparity(imgL,imgR);
}

/************************************************************************/
bool ObjectReconstr::updateModule()
{
    if (!updateCloud())
        return false;
    switch(currentState)
    {

    case STATE_WAIT:
        return true;

    case STATE_RECONSTRUCT:
        {
            recRoutine.resetClouds();

            if (!updateCloud())
                return false;
                
            printf("Cloud reconstructed\n");

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp=recRoutine.getPointCloudComplete();
            if (tmp->size()==0)
            {
                printf("Empty cloud\n");
                middlex=-1;
                middley=-1;
                currentState=STATE_WAIT;
			    SurfaceMeshWithBoundingBox &pointCloudOnPort=pointCloudPort.prepare();
				pointCloudOnPort.mesh.points.clear();  
				pointCloudOnPort.mesh.rgbColour.clear();
				pointCloudPort.write();
                return true;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			filter(tmp,cloud);

            SurfaceMeshWithBoundingBox &pointCloudOnPort=pointCloudPort.prepare();
            pointCloudOnPort.mesh.points.clear();  
            pointCloudOnPort.mesh.rgbColour.clear();
            for (unsigned int i=0; i<cloud->width; i++)
            {
                pointCloudOnPort.mesh.points.push_back(PointXYZ(cloud->at(i).x,cloud->at(i).y, cloud->at(i).z));
                pointCloudOnPort.mesh.rgbColour.push_back(RGBA(cloud->at(i).rgba));
            }
            if (computeBB)
            {
                cout << " computing BB " << endl;
                boundingBox=MinimumBoundingBox::getMinimumBoundingBox(cloud);
                pointCloudOnPort.boundingBox=boundingBox.getBoundingBox();
            }
            pointCloudPort.write();

            if (write)
                savePointsPly(tmp, fileName);

            if (visualizationOn)
                currentState=STATE_VISUALIZE;
            else
                currentState=STATE_WAIT;

            middlex=-1;
            middley=-1;

            return true;
        }

    case STATE_VISUALIZE:
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=recRoutine.getPointCloud();
            if (visualizationOn)
            {
                /*
                boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
                tmpViewer->setBackgroundColor (0, 0, 0);
                if (computeBB)
                {
                    boundingBox.drawBoundingBox(tmpViewer);
                }
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudComplete;
                cloudComplete=recRoutine.getPointCloudComplete();
                visualize(tmpViewer, cloudComplete);
                */
                visThrd->updateCloud(cloud);
                if (computeBB)
                {
                    cout << "Plotting BB " << endl;
                    visThrd->addBoundingBox(true);
                }
            }
            currentState=STATE_WAIT;
        }
    }

    return true;
}

/************************************************************************/
void ObjectReconstr::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (cloud_in->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

/************************************************************************/
bool ObjectReconstr::interruptModule()
{
    closing=true;
    rpc.interrupt();

    imagePortInLeft.interrupt();
    imagePortInRight.interrupt();

    pointCloudPort.interrupt();
    segmentationPort.interrupt();

    return true;
}

/************************************************************************/
double ObjectReconstr::getPeriod()
{
    return 0.1;
}

/************************************************************************/
bool ObjectReconstr::respond(const Bottle& command, Bottle& reply) 
{
    if (command.get(0).asString()=="help")
    {
        reply.addVocab(ACK);
        reply.addString("To obtain a reconstruction, the module needs a segmentation algorithm that returns the set of pixels belonging to the object");
        reply.addString("It typically uses the graphBasedSegmentation module");
        reply.addString("Provide a point of the object first, in the format x y, for instance clicking on the segmentation image.");
        reply.addString("Then, when you send the command 3Drec, you will be provided with the reconstructed object along with the minimum enclosing bounding box on the output port of the module -- typically /objectReconstr/mesh:o.");
        reply.addString("If you also want to visualize the result, the command is 3Drec visualize.");
        reply.addString("If you want to reconstruct a single pixel, the command is get point (x y).");
        return true;
    }

    if (command.get(0).asString()=="set")
    {
        if (command.get(1).asString()=="write")
        {
            if (command.size()>2)
            {
                if (command.get(2).asString()=="on")
                {
                    reply.addVocab(ACK);
                    write=true;
                }
                else if (command.get(2).asString()=="off")
                {
                    write=false;
                    reply.addVocab(ACK);
                }
                else
                    reply.addVocab(NACK);
                return true;
            }
        }
        else
        {
            reply.addVocab(NACK);
            return true;
        }
    }
    if (command.get(0).asString()=="3Drec")
    {
        if (middlex==-1 || middley==-1)
        {
            reply.addVocab(NACK);
            reply.addString("I need a pixel of the object");
            return true;
        }

        currentState=STATE_RECONSTRUCT;

        visualizationOn=false;

        if (command.size()==2)
            if (command.get(1).asString()=="visualize")
                visualizationOn=true;

        reply.addVocab(ACK);
        return true;
    }

    if (command.get(0).asString()=="name")
    {
        if (command.size()>=2)
        {
            fileName=command.get(1).asString().c_str();
            reply.addVocab(ACK);
 	    }
        else
        {
            reply.addVocab(NACK);
	        reply.addString("No name was provided");
        }

        return true;
    }

    if (command.get(0).asString()=="get")
    {
        if (command.get(1).asString()=="point" && command.size()==3)
        {
            if (currentState!=STATE_RECONSTRUCT)
            {
                IplImage* imgL;
                IplImage* imgR;

                ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(true);
                ImageOf<PixelRgb> *tmpR = imagePortInRight.read(true);

                if(tmpL!=NULL && tmpR!=NULL)
                {
                    imgL= (IplImage*) tmpL->getIplImage();
                    imgR= (IplImage*) tmpR->getIplImage();
                }
                else
                {
                    reply.addVocab(NACK);
                    return true;
                }

                yarp::sig::Vector point2D(2);
                Bottle *pixel=command.get(2).asList();
                point2D[0]=pixel->get(0).asDouble();
                point2D[1]=pixel->get(1).asDouble();

                yarp::sig::Vector point3D(3);

                bool done=recRoutine.triangulateSinglePoint(imgL,imgR,point2D,point3D);

                if (done)
                {
                    Bottle &result=reply.addList();
                    result.addDouble(point3D[0]);
                    result.addDouble(point3D[1]);
                    result.addDouble(point3D[2]);
                }
                else
                    reply.addVocab(NACK);
            }
            else
            {
                reply.addVocab(NACK);
                reply.addString("Still processing");
            }
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            return true;
        }
    }

    if (command.size()==2)
    {
        if (command.get(0).asInt()!=0 && command.get(1).asInt()!=0)
        {
            middlex=(double)command.get(0).asInt();
            middley=(double)command.get(1).asInt();
            reply.addVocab(ACK);
            return true;
        }
        else
        {
            reply.addVocab(NACK);
            return true;
        }
    }

    reply.addVocab(NACK);
    return true;
}

/************************************************************************/
/*
void ObjectReconstr::visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> tmpViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    string id="Object Cloud";
    tmpViewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id);
    tmpViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    tmpViewer->initCameraParameters();
    while (!tmpViewer->wasStopped())
    {
        if (closing)
        {
            tmpViewer->close();
            break;
        }
        tmpViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    tmpViewer->removePointCloud(id);
}

*/
