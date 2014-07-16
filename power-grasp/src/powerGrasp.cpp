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

#include "powerGrasp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;
using namespace iCub::learningmachine;
using namespace iCub::data3D;
using namespace pcl::io;
using namespace pcl;

/************************************************************************/
PowerGrasp::PowerGrasp() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), 
    cloudxyz(new pcl::PointCloud<pcl::PointXYZ>),
    normals (new pcl::PointCloud <pcl::Normal>)
{
    modality=MODALITY_AUTO;
    path="";

    chosenPoint.resize(3,0.0);
    chosenNormal.resize(3,0.0);
    chosenPixel.resize(2,0.0);
    chosenPoint[0]=-0.3;
    chosenPoint[2]=0.3;

    offsetR.resize(3,0.0);
    offsetL.resize(3,0.0);

    resetBools();
    train=false;
    visualize=true;
    straight=true;
    rightBlocked=false;
    leftBlocked=false;
    rightDisabled=false;
    leftDisabled=false;
    fromFileFinished=false;
    filterCloud=true;
    graspSpecificPoint=false;
    writeCloud=false;
    nFile=0;

    data.cloud=cloud;
    data.normals=normals;
    data.boundingBox=&boundingBox;
    data.goodPointsIndexes=&rankIndices;

    mutex.wait();
    currentState=STATE_WAIT;
    mutex.post();
    visualizationThread=new VisualizationThread(data);
}

/************************************************************************/
void PowerGrasp::configureGeneralInfo(ResourceFinder &rf)
{
    radiusSearch=rf.check("radiusSearch",Value(0.045)).asDouble();
    numberOfBestPoints=rf.check("numberOfBestPoints",Value(10)).asInt();
    bestCurvature=(float)rf.check("curvature",Value(0.005)).asDouble();
    handSize=rf.check("handSize",Value(0.08)).asDouble();
    fingerSize=rf.check("fingerSize",Value(0.08)).asDouble();
    string useFile=rf.check("fromFile",Value("false")).asString().c_str();
    string useLearning=rf.check("useLearning",Value("false")).asString().c_str();
    fromFile=(useFile=="true");
    testWithLearning=(useLearning=="true");
    posx=rf.check("x",Value(0)).asInt();
    posy=rf.check("y",Value(0)).asInt();
    visualizationThread->setPosition(posx, posy);
    sizex=rf.check("w",Value(320)).asInt();
    sizey=rf.check("h",Value(240)).asInt();
    visualizationThread->setSize(sizex, sizey);

    if (!rf.check("disableRight"))
        orientationThreadRight=new OrientationThread();
    else
    {
        rightDisabled=true;
        printf("**********RIGHT ARM DISABLED**************\n");
    }
    if (!rf.check("disableLeft"))
        orientationThreadLeft=new OrientationThread();
    else
    {
        leftDisabled=true;
        printf("**********LEFT ARM DISABLED**************\n");
    }
}

/************************************************************************/
void PowerGrasp::configureSVM(Bottle &bottleSVM)
{
    // lssvm: default values
    scalerIn.setLowerBoundIn(0.0);
    scalerIn.setUpperBoundIn(0.03);
    scalerIn.setLowerBoundOut(0.0);
    scalerIn.setUpperBoundOut(1.0);

    scalerOut.setLowerBoundIn(0.0);
    scalerOut.setUpperBoundIn(0.6);
    scalerOut.setLowerBoundOut(0.0);
    scalerOut.setUpperBoundOut(1.0);

    machine.setDomainSize(1);
    machine.setCoDomainSize(1);
    machine.setC(20.0);
    machine.getKernel()->setGamma(100.0);

    if (!bottleSVM.isNull())
    {
        if (bottleSVM.check("c"))
            machine.setC(bottleSVM.find("c").asDouble());

        if (bottleSVM.check("gamma"))
            machine.getKernel()->setGamma(bottleSVM.find("gamma").asDouble());

        if (Bottle *v=bottleSVM.find("in_bounds").asList())
        {
            if (v->size()>=2)
            {
                scalerIn.setLowerBoundIn(v->get(0).asDouble());
                scalerIn.setUpperBoundIn(v->get(1).asDouble());
            }
        }

        if (Bottle *v=bottleSVM.find("out_bounds").asList())
        {
            if (v->size()>=2)
            {
                scalerOut.setLowerBoundIn(v->get(0).asDouble());
                scalerOut.setUpperBoundIn(v->get(1).asDouble());
            }
        }

        if (Bottle *v=bottleSVM.find("machine").asList())
            testWithLearningEnabled=machine.fromString(v->toString().c_str());
        else
            testWithLearningEnabled=false;
    }
    else
        testWithLearningEnabled=false;
}

/************************************************************************/
bool PowerGrasp::configure(ResourceFinder &rf)
{
    configureGeneralInfo(rf);

    string name=rf.check("name",Value("power-grasp")).asString().c_str();

    int nAngles=rf.check("nAngles",Value(50)).asInt();
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string rightArm="right_arm";
    string leftArm="left_arm";

    if (rightDisabled && leftDisabled)
    {
        printf("Both arms disabled\n");
        return false;
    }
    
    bool rightOK=true;
    if (!rightDisabled)
        rightOK=orientationThreadRight->open(name,rightArm,robot,nAngles);
    bool leftOK=true;
    if (!leftDisabled)
        leftOK=orientationThreadLeft->open(name,leftArm,robot,nAngles);
    if (!rightOK || !leftOK)
    {
        printf("Orientation threads did not open\n");
        return false;
    }
    
    outputDir=rf.find("outputDir").asString().c_str();

    if (fromFile)
    {
        path=rf.find("path").asString().c_str();
        if (path=="")
        {
            printf("Please specify a folder to find the .ply file\n");
            return false;
        }
    }

    Bottle *pR=rf.find("offsetR").asList();
    if (pR->size()>0)
    {
        for (int i=0; i<pR->size(); i++)
            offsetR[i]=pR->get(i).asDouble();
    }

    Bottle *pL=rf.find("offsetL").asList();
    if (pL->size()>0)
    {
        for (int i=0; i<pL->size(); i++)
            offsetR[i]=pL->get(i).asDouble();
    }

    rpc.open(("/"+name+"/rpc").c_str());
    attach(rpc);

    if (!meshPort.open(("/"+name+"/mesh:i").c_str()))
        return false;

    if (!areCmdPort.open(("/"+name+"/are/cmd:o").c_str()))
        return false;

    if (!reconstructionPort.open(("/"+name+"/reconstruction").c_str()))
        return false;

    if (!depth2kin.open(("/"+name+"/depth2kin:o").c_str()))
        return false;
   
    Bottle &bMachine=rf.findGroup("lssvm");
    configureSVM(bMachine);

    if (!rightDisabled)
        orientationThreadRight->start();
    if (!leftDisabled)    
        orientationThreadLeft->start();

    return true;
}

/************************************************************************/
bool PowerGrasp::interruptModule()
{
    eventRpc.signal();
    reconstructionPort.interrupt();
    areCmdPort.interrupt();
    depth2kin.interrupt();
    meshPort.interrupt();
    rpc.interrupt();

    return true;
}

/************************************************************************/
bool PowerGrasp::close()
{
    reconstructionPort.close();
    areCmdPort.close();
    depth2kin.close();
    meshPort.close();
    rpc.close();

    if (visualizationThread->isRunning())
        visualizationThread->stop();
    delete visualizationThread;

    if (!rightDisabled)
    {
        orientationThreadRight->stop();
        delete orientationThreadRight;
    }
    if (!leftDisabled)
    {    
        orientationThreadLeft->stop();
        delete orientationThreadLeft;
    }

    return true;
}

/************************************************************************/
void PowerGrasp::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (cloud_in->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

/************************************************************************/
void PowerGrasp::write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const int nFile)
{
    stringstream ss;
    ss << nFile;
    string str = ss.str();
    string filename=outputDir+"/cloud"+str+".ply";
    ofstream cloudFile;
    cloudFile.open (filename.c_str());
    cloudFile << "ply\n";
    cloudFile << "format ascii 1.0\n";
    cloudFile << "element vertex " << cloud_in->size() << "\n";
    cloudFile << "property float x\n";
    cloudFile << "property float y\n";
    cloudFile << "property float z\n";
    cloudFile << "property uchar diffuse_red\n";
    cloudFile << "property uchar diffuse_green\n";
    cloudFile << "property uchar diffuse_blue\n";
    cloudFile << "end_header\n";
    for (int i=0; i<cloud_in->size(); i++)
    {
        cloudFile << cloud_in->at(i).x << " " << cloud_in->at(i).y << " " << cloud_in->at(i).z << " " << (int)cloud_in->at(i).r << " " << (int)cloud_in->at(i).g << " " << (int)cloud_in->at(i).b << "\n";
    }
    cloudFile.close();
}

/************************************************************************/
void PowerGrasp::fromSurfaceMesh (const SurfaceMeshWithBoundingBox& msg)
{
    cloud->clear();
    cloudxyz->clear();

    for (size_t i = 0; i<msg.mesh.points.size(); ++i)
    {
        PointXYZRGB pointrgb;
        pointrgb.x=msg.mesh.points.at(i).x;
        pointrgb.y=msg.mesh.points.at(i).y;
        pointrgb.z=msg.mesh.points.at(i).z;
        if (i<msg.mesh.rgbColour.size())
        {
            int32_t rgb= msg.mesh.rgbColour.at(i).rgba;
            pointrgb.rgba=rgb;
            pointrgb.r = (rgb >> 16) & 0x0000ff;
            pointrgb.g = (rgb >> 8)  & 0x0000ff;
            pointrgb.b = (rgb)       & 0x0000ff;
        }
        else
            pointrgb.rgb=0;

        pcl::PointXYZ point;
        point.x=pointrgb.x;
        point.y=pointrgb.y;
        point.z=pointrgb.z;
        cloudxyz->push_back(point);
        cloud->push_back(pointrgb);
    }

    boundingBox.setBoundingBox(msg.boundingBox);
}

/************************************************************************/
bool PowerGrasp::fillCloudFromFile()
{
    struct dirent *entry;
    DIR *dp;
     
    dp = opendir(path.c_str());
    if (dp == NULL) 
    {
        perror("opendir");
        return false;
    }

    while((entry = readdir(dp)))
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        else
            break;
    }

    if (entry->d_name!=NULL)
    {
        pcl::PointCloud<PointXYZRGB>::Ptr cloud_in_rgb (new pcl::PointCloud<PointXYZRGB>);

        string root=path+"/";
        string name=entry->d_name;
        string file=root+name;

        if (loadPLYFile(file.c_str(), *cloud_in_rgb) == -1) 
            return false;

        if (filterCloud)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
            filter(cloud_in_rgb,cloud_in_filtered);
            addPointCloud(cloud_in_filtered);
        }
        else
            addPointCloud(cloud_in_rgb);
    }
    closedir(dp);
    return true;
}

/************************************************************************/
int PowerGrasp::findIndexFromCloud(const yarp::sig::Vector &point)
{
    double minNorm=1e10;
    int index=-1;
    for (int i=0; i<cloud->size(); i++)
    {
        yarp::sig::Vector currentPoint=vectorFromCloud(i);

        if (norm(point-currentPoint)<minNorm)
        {
            index=i;
            minNorm=norm(point-currentPoint);
        }
    }
    return index;
}

/************************************************************************/
void PowerGrasp::normalEstimation()
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setRadiusSearch (radiusSearch);
    normal_estimator.setInputCloud (cloudxyz);
    normal_estimator.compute (*normals);
}

/************************************************************************/
void PowerGrasp::rankPoints()
{              
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloudxyz);

    yarp::sig::Vector center=boundingBox.getCenter();

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int nGoodPoints=0;

    bool done=false;
    //Choose best points
    int minNeighbohrs=300;
    while(!done)
    {
        rankIndices.clear();
        rankScores.clear();
	    for (int i=0; i<cloudxyz->size(); i++)
	    {
	        int index=i;
            int n_neighbohrs=kdtree.radiusSearch(cloudxyz->at(index),radiusSearch,pointIdxRadiusSearch,pointRadiusSquaredDistance);
            if (n_neighbohrs>minNeighbohrs)
            {
                if (normalPointingOut(index,center))
                {
                    nGoodPoints++;
                    double score=scoreFunction(index);
                    insertElement(score,index);
                }
            }
        }
        if (nGoodPoints<numberOfBestPoints && minNeighbohrs>0)
        {
            minNeighbohrs-=10;
            nGoodPoints=0;
        }
        else
        {
            done=true;
            break;
        }
    }
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::vectorFromNormal(const int index)
{
    yarp::sig::Vector currNormal(3);
    currNormal[0]=normals->at(index).normal_x;
    currNormal[1]=normals->at(index).normal_y;
    currNormal[2]=normals->at(index).normal_z;

    return currNormal;
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::vectorFromCloud(const int index)
{
    yarp::sig::Vector currPoint(3);
    currPoint[0]=cloudxyz->at(index).x;
    currPoint[1]=cloudxyz->at(index).y;
    currPoint[2]=cloudxyz->at(index).z;

    return currPoint;
}

/************************************************************************/
void PowerGrasp::chooseCandidatePoints()
{             
    if (train)
    {
        double tmp=Random::uniform();
        currentCurvature=(float)fmod(tmp,maxCurvature);
        printf("Chosen curvature %g\n", currentCurvature);
        printf("Max curvature %g\n", maxCurvature);
    }
    else
        currentCurvature=bestCurvature;

    currentModality=modality;

    if (currentModality==MODALITY_AUTO)   
        manageModality();

    rankPoints();
            
    string smodality;
    if (currentModality==MODALITY_CENTER)
        smodality="center";
    else if (currentModality==MODALITY_LEFT)
        smodality="left";
    else if (currentModality==MODALITY_RIGHT)
        smodality="right";
    else
        smodality="top";
                
    printf("modality chosen %s\n", smodality.c_str());
}

/************************************************************************/
void PowerGrasp::startVisualization()
{
    data.chosenPoint=chosenPoint;
    data.chosenOrientation=chosenOrientation;
    data.hand=chosenHand;

    visualizationThread->start();
}

/************************************************************************/
bool PowerGrasp::updateModule()
{
    mutex.wait();
    if ((fromFile && !fromFileFinished) || (currentState==STATE_ESTIMATE))
    {
        mutex.post();
        double totTime=Time::now();
        if (fromFile)
        {
            if (!fillCloudFromFile())
            {
                printf("Error in reading the .ply file\n");
                return false;
            }
            boundingBox=MinimumBoundingBox::getMinimumBoundingBox(cloud);
        }    
        else 
        {
            mutex.wait();
            if (currentState==STATE_ESTIMATE)
            {
                SurfaceMeshWithBoundingBox *receivedMesh=meshPort.read(false);

                if (receivedMesh!=NULL)
                    fromSurfaceMesh(*receivedMesh);
                else
                {
                    mutex.post();
                    return true;
                }
            }
            mutex.post();
        }

        if (writeCloud && outputDir!="")
        {
            write(cloud,nFile);
            nFile++;
        }

        computeDim();

        normalEstimation();

        rankIndices.clear();
        rankScores.clear();

        if (graspSpecificPoint)
        {
            int pointIndex;
            pointIndex=findIndexFromCloud(chosenPoint);
            chosenNormal=vectorFromNormal(pointIndex);

            double score=1.0;
            insertElement(score,pointIndex);
        }
        else
            chooseCandidatePoints();

        Matrix designedOrientation=eye(4,4);
        winnerIndex=-1;

        string hand=chooseBestPointAndOrientation(winnerIndex,designedOrientation);

        yarp::sig::Vector tmp=dcm2axis(designedOrientation);

        if (hand==NO_HAND || tmp.size()==0)
        {
            mutex.wait();
            currentState=STATE_WAIT;
            mutex.post();
            eventRpc.signal();
            if (fromFile)
                fromFileFinished=true;
            return true;
        }

        chosenHand=hand;
        chosenOrientation=designedOrientation;

        if (!graspSpecificPoint)
        {
            chosenPoint=vectorFromCloud(winnerIndex);
            chosenNormal=vectorFromNormal(winnerIndex);
        }

        if (visualize)
            startVisualization();

        printf("Chosen point: %s\n", chosenPoint.toString().c_str());
        printf("Chosen orientation: %s\n", tmp.toString().c_str());
        printf("Chosen hand: %s\n", chosenHand.c_str());

        if (!fromFile)
        {
            readyToGrasp=true;
            if (straight)
            {
                mutex.wait();
                currentState=STATE_GRASP;
                mutex.post();
            }
            else
            {
                mutex.wait();
                currentState=STATE_WAIT;
                mutex.post();
            }
        }
        else
            fromFileFinished=true;

        graspSpecificPoint=false;
        printf("Tot time %g\n", Time::now()-totTime);
        return true;
    }
    mutex.post();

    mutex.wait();
    if (currentState==STATE_GRASP)
    {
        mutex.post();
        askToGrasp();        
        mutex.wait();
        currentState=STATE_WAIT;
        mutex.post();
        eventRpc.signal();
        return true;
    }
    mutex.post();

    return true;
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::assignIndexToAxes(double &anglez)
{
    yarp::sig::Vector indices(3); indices=-1;
    yarp::sig::Vector refz(3); refz=0.0; refz[2]=1.0;
    yarp::sig::Vector refy(3); refy=0.0; refy[1]=1.0;
    yarp::sig::Vector x,y,z;
    boundingBox.getAxis(x,y,z);
    x=x/norm(x);
    y=y/norm(y);
    z=z/norm(z);
    double dotzx=fabs(dot(x,refz));
    double dotzy=fabs(dot(y,refz));
    double dotzz=fabs(dot(z,refz));
    double dotyx=fabs(dot(x,refy));
    double dotyy=fabs(dot(y,refy));
    double dotyz=fabs(dot(z,refy));

    if (dotzx>dotzy && dotzx>dotzz)
    {
        indices[2]=0;
        anglez=acos(dotzx);
        if (dotyy>dotyz)
        {
            indices[0]=2;
            indices[1]=1;
        }
        else
        {
            indices[0]=1;
            indices[1]=2;
        }
    }
    else if (dotzy>dotzx && dotzy>dotzz)
    {
        indices[2]=1;
        anglez=acos(dotzy);
        if (dotyx>dotyz)
        {
            indices[0]=2;
            indices[1]=0;
        }
        else
        {
            indices[0]=0;
            indices[1]=2;
        }
    }
    else
    {
        indices[2]=2;
        anglez=acos(dotzz);
        if (dotyx>dotyy)
        {
            indices[0]=1;
            indices[1]=0;
        }
        else
        {
            indices[0]=0;
            indices[1]=1;
        }
    }

    return indices;
}

/************************************************************************/
void PowerGrasp::manageModality()
{
    yarp::sig::Vector center=boundingBox.getCenter();
    yarp::sig::Vector dim=boundingBox.getDim();
    
    double anglez;
    yarp::sig::Vector indices=assignIndexToAxes(anglez);

    yarp::sig::Vector right(3); right=0.0;
    right[0]=-0.3; right[1]=0.3;
    yarp::sig::Vector left(3); left=0.0;
    left[0]=-0.3; left[1]=-0.3;

    double distRight=norm(right-center);
    double distLeft=norm(left-center);

    if (distRight/distLeft>1.8)
    {
        printf("left closer\n");
        blockRightTmp=true;
    }
    else if (distLeft/distRight>1.8)
    {
        printf("right closer\n");
        blockLeftTmp=true;
    }

    if (dimz<handSize/2 || dimz<fingerSize)
    {
        printf("object too small to be taken from side %g\n", dimz);
        currentModality=MODALITY_TOP;
    }
    else if ((dim[indices[2]]/dim[indices[1]]>1.8) && (dim[indices[2]]/dim[indices[0]]>1.8) && (anglez<60.0*M_PI/180.0))
    {
        printf("object very tall, going from side\n");
        if (distRight/distLeft>1.3 || rightBlocked || rightDisabled || blockRightTmp)
            currentModality=MODALITY_LEFT;
        else if (distLeft/distRight>1.3 || leftBlocked || leftDisabled || blockLeftTmp)
            currentModality=MODALITY_RIGHT;
        else
            currentModality=(int)(Random::uniform(0,1)+0.5);
    }
    else if ((dim[indices[0]]/dim[indices[1]]>1.5 && dim[indices[0]]/dim[indices[2]]>1.5) || (dim[indices[1]]/dim[indices[0]]>1.5 && dim[indices[1]]/dim[indices[2]]>1.5))
    {
        printf("center\n");
        currentModality=MODALITY_CENTER;
    }
    else 
    {
        printf("random\n");
        Random::seed((int)Time::now()*1000);
        if (rightDisabled || rightBlocked)
        {
            int tmp=(int)(Random::uniform(0,1)+0.5);
            if (tmp==1)
                currentModality=MODALITY_TOP;
            else
                currentModality=MODALITY_LEFT;
        }
        else if (leftDisabled || leftBlocked)
        {
            int tmp=(int)(Random::uniform(0,1)+0.5);
            if (tmp==1)
                currentModality=MODALITY_TOP;
            else
                currentModality=MODALITY_RIGHT;
        }
        else
            currentModality=(int)(Random::uniform(0,2)+0.5);
    }
    if (currentModality==MODALITY_LEFT)
        blockRightTmp=true;
    if (currentModality==MODALITY_RIGHT)
        blockLeftTmp=true;
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::computeApproachVector(const yarp::sig::Vector &chosenPoint)
{
    yarp::sig::Vector approachVector(4); approachVector=0.0;
    yarp::sig::Vector center=boundingBox.getCenter();

    if (currentModality==MODALITY_LEFT || currentModality==MODALITY_RIGHT)
    {
        if (chosenHand==LEFT_HAND)
        {
            approachVector[2]=0.08;
            approachVector[3]=40.0;
        }
        else
        {
            approachVector[2]=-0.08;
            approachVector[3]=-40.0;
        }
    }
    else if (currentModality==MODALITY_TOP)
    {
        //if the point is much further than the center of the x, add a displacement back along x
        if (chosenPoint[0]-center[0]<-handSize/2)
            approachVector[0]=-0.08;
        if (chosenHand==LEFT_HAND)
        {
            approachVector[2]=0.08;
            approachVector[3]=15.0;
        }
        else
        {
            approachVector[2]=-0.08;
            approachVector[3]=-15.0;
        }
    }

    return approachVector;
}

/************************************************************************/
void PowerGrasp::askToGrasp()
{
    yarp::sig::Vector approachVector=computeApproachVector(chosenPoint);
    Bottle cmd,reply;
    if (depth2kin.getOutputCount()>0)
    {
        cmd.addString("getPoint");
        cmd.addString(chosenHand.c_str());
        cmd.addDouble(chosenPoint[0]);
        cmd.addDouble(chosenPoint[1]);
        cmd.addDouble(chosenPoint[2]);

        depth2kin.write(cmd,reply);
        if (reply.get(0).asString()=="ok")
            chosenPoint=pointFromBottle(reply,1);
    }

    cmd.clear(); reply.clear();
    printf("Chosen point depth2kin %g %g %g\n", chosenPoint[0],chosenPoint[1],chosenPoint[2]);

    yarp::sig::Vector tmp=dcm2axis(chosenOrientation);

    printf("dimz %g fingerSize %g\n", dimz, fingerSize);

    if (currentModality=MODALITY_TOP && dimz<fingerSize)
    {
        double diff=fingerSize-dimz;
        chosenPoint+=(chosenNormal*diff);
        printf("dbg: chosen point %s\n", chosenPoint.toString().c_str());
    }

    cmd.addString("grasp");
    Bottle &point=cmd.addList();
    point.addDouble(chosenPoint[0]);
    point.addDouble(chosenPoint[1]);
    point.addDouble(chosenPoint[2]);
    point.addDouble(tmp[0]);
    point.addDouble(tmp[1]);
    point.addDouble(tmp[2]);
    point.addDouble(tmp[3]);
    cmd.addString(chosenHand.c_str());
    Bottle &approach=cmd.addList();
    approach.addString("approach");
    Bottle &vector=approach.addList();
    vector.addDouble(approachVector[0]);
    vector.addDouble(approachVector[1]);
    vector.addDouble(approachVector[2]);
    vector.addDouble(approachVector[3]);

    if (areCmdPort.getOutputCount()>0)
    {
        areCmdPort.write(cmd,reply);
        if (reply.get(0).asString()==ACK)
            grasped=true;
    }
    readyToGrasp=false;
}

/************************************************************************/
void PowerGrasp::resetBools()
{
    grasped=false;
    readyToGrasp=false;
    blockRightTmp=false;
    blockLeftTmp=false;
    noResult=false;
    tooFar=false;
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::pointFromBottle(const Bottle &bot, const int index)
{
    yarp::sig::Vector vector(3);
    vector[0]=bot.get(index).asDouble();
    vector[1]=bot.get(index+1).asDouble();
    vector[2]=bot.get(index+2).asDouble();

    return vector;
}

/************************************************************************/
bool PowerGrasp::get3DPoint(const yarp::sig::Vector &point2D, yarp::sig::Vector &point3D)
{
    Bottle cmd; Bottle reply;
    cmd.clear(); reply.clear();
    cmd.addString("get");
    cmd.addString("point");
    cmd.addDouble(point2D[0]);
    cmd.addDouble(point2D[1]);

    if (reconstructionPort.getOutputCount()>0)
        reconstructionPort.write(cmd,reply);

    if (reply.size()>0 && reply.get(0).asString()!=NACK)
    {
        point3D=pointFromBottle(reply,0);
        return true;
    }
    else
        return false;
}

/************************************************************************/
bool PowerGrasp::respond(const Bottle& command, Bottle& reply) 
{
    string tag_0=command.get(0).asString().c_str();
    if (tag_0=="set")
    {
        if (command.size()<3)
        {
            reply.addString("command not recognized");
            return true;
        }
        string tag_1=command.get(1).asString().c_str();
        if (tag_1=="visualization")
        {
            if (command.get(2).asString()=="on")
                visualize=true;
            else
                visualize=false;
            reply.addString(ACK);
            return true;
        }
        else if (tag_1=="x")
        {
            if (command.size()>1)
            {
                posx=command.get(1).asInt();
                visualizationThread->setPosition(posx,posy);
                reply.addString(ACK);
            }
            else
                reply.addString(NACK);
            return true;
        }
        else if (tag_1=="y")
        {
            if (command.size()>1)
            {
                posy=command.get(1).asInt();
                visualizationThread->setPosition(posx,posy);
                reply.addString(ACK);
            }
            else
                reply.addString(NACK);
            return true;
        }
        else if (tag_1=="w")
        {
            if (command.size()>1)
            {
                sizex=command.get(1).asInt();
                visualizationThread->setSize(sizex,sizey);
                reply.addString(ACK);
            }
            else
                reply.addString(NACK);
            return true;
        }
        else if (tag_1=="h")
        {
            if (command.size()>1)
            {
                sizey=command.get(1).asInt();
                visualizationThread->setSize(sizex,sizey);
                reply.addString(ACK);
            }
            else
                reply.addString(NACK);
            return true;
        }
        else if (tag_1=="train")
        {
            if (command.get(2).asString()=="on")
            {
                train=true;
                testWithLearning=false;
            }
            else
                train=false;
            reply.addString(ACK);
            return true;
        }
        else if (tag_1=="testWithSVM")
        {
            if (command.get(2).asString()=="on")
            {
                if (testWithLearningEnabled)
                {
                    testWithLearning=true;
                    train=false;
                }
                else
                    printf("SVM machine not set\n");
            }
            else
                testWithLearning=false;
            reply.addString(ACK);
            return true;
        }
        else if (tag_1=="offsetR")
        {
            if (command.size()<5)
            {
                reply.addString("nack, check offset size");
                return true;
            }
            else
            {
                offsetR=pointFromBottle(command,2);
                reply.addString(ACK);
                return true;
            }
        }
        else if (tag_1=="offsetL")
        {
            if (command.size()<5)
            {
                reply.addString("nack, check offset size");
                return true;
            }
            else
            {
                offsetL=pointFromBottle(command,2);
                reply.addString(ACK);
                return true;
            }
        }
        else if (tag_1=="modality")
        {
            if (command.get(2).asString()=="right")
                modality=MODALITY_RIGHT;
            else if (command.get(2).asString()=="left")
                modality=MODALITY_LEFT;
            else if (command.get(2).asString()=="center")
                modality=MODALITY_CENTER;
            else if (command.get(2).asString()=="top")
                modality=MODALITY_TOP;
            else
                modality=MODALITY_AUTO;
            reply.addString(ACK);
            return true;
        }
        else if (tag_1=="filter")
        {
            if (command.get(2).asString()=="on")
                filterCloud=true;
            else
                filterCloud=false;
            reply.addString(ACK);
            return true;
        }
        else if (tag_1=="write")
        {
            if (command.get(2).asString()=="on")
                writeCloud=true;
            else
                writeCloud=false;
            reply.addString(ACK);
            return true;
        }
        else
        {
            reply.addString(NACK);
            return true;
        }
    }
    else if (tag_0=="help")
    {
        reply.addString("set visualization on/off");
        reply.addString("set x");
        reply.addString("set y");
        reply.addString("set w");
        reply.addString("set h");
        reply.addString("set train on/off");
        reply.addString("set testWithSVM on/off");
        reply.addString("set offsetL x y z");
        reply.addString("set offsetR x y z");
        reply.addString("set modality right/left/top/center");
        reply.addString("set filter on/off");
        reply.addString("set write on/off");
        reply.addString("block right/left");
        reply.addString("unblock right/left");
        reply.addString("grasp (x y) [wait]");
        reply.addString("go");
        reply.addString("dont");
        reply.addString("isGrasped");
        return true;
    }
    else if (tag_0=="go")
    {
        if (readyToGrasp)
        {
            mutex.wait();
            currentState=STATE_GRASP;
            mutex.post();
            eventRpc.reset();
            eventRpc.wait();
            reply.addString(ACK);
        }
        else
            reply.addString(NACK);
        return true;
    }
    else if (tag_0=="block")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=true;
            else
                leftBlocked=true;
            reply.addString(ACK);
        }
        else
            reply.addString(NACK);
        return true;
    }
    else if (tag_0=="unblock")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=false;
            else
                leftBlocked=false;
            reply.addString(ACK);
        }
        else
            reply.addString(NACK);
        return true;
    }
    else if (tag_0=="isGrasped")
    {
        string r=grasped?ACK:NACK;
        reply.addString(r.c_str());
        return true;
    }
    else if (tag_0=="dont")
    {
        resetBools();
        mutex.wait();
        currentState=STATE_WAIT;
        mutex.post();
        reply.addString(ACK);
        return true;
    }
    else if (tag_0=="grasp")
    {
        mutex.wait();
        if (currentState==STATE_ESTIMATE || currentState==STATE_GRASP)
        {
            mutex.post();
            reply.addString(NACK);
            return true;
        }
        mutex.post();

        Bottle *pos=command.get(1).asList();
        chosenPixel[0]=pos->get(0).asInt();
        chosenPixel[1]=pos->get(1).asInt();

        Bottle cmd1;
        reply.clear();
        cmd1.addInt(chosenPixel[0]);
        cmd1.addInt(chosenPixel[1]);

        if (reconstructionPort.getOutputCount()>0)
            reconstructionPort.write(cmd1,reply);

        if (visualizationThread->isRunning())
            visualizationThread->stop();

        resetBools();

        if (train && command.size()>1)
            currentCurvature=(float)command.get(1).asDouble();

        if (pos->size()>2)
        {
            if (pos->get(2).asString()=="point")
            {
                if (get3DPoint(chosenPixel,chosenPoint))
                    graspSpecificPoint=true;
                else
                {
                    reply.clear();
                    reply.addString(NACK);
                    return true;
                }
            }
        }

        reply.clear();
        Bottle tmp;
        tmp.clear();
        tmp.addString("3Drec");

        if (reconstructionPort.getOutputCount()>0)
            reconstructionPort.write(tmp,reply);

        if (reply.size()>0 && reply.get(0).asString()==ACK)
        {
            reply.clear();
            mutex.wait();
            currentState=STATE_ESTIMATE;
            mutex.post();
            straight=true;
            if (command.size()>2)
                straight=(command.get(2).asString()!="wait");
            if (straight)
            {
                eventRpc.reset();
                eventRpc.wait();
            }
            if (noResult)
            {
                reply.addString(NACK);
                reply.addString("no_result");
                noResult=false;
            }
            else if (tooFar)
            {
                reply.addString(NACK);
                reply.addString("too_far");
                tooFar=false;
            }
            else
            {
                reply.addString(ACK);
            }
        }
        else
            reply.addString(NACK);
      
        return true;
    }
    reply.addString(NACK);
    return true;
}

/************************************************************************/
yarp::sig::Vector PowerGrasp::findBiggestAxis(int &ind)
{
    yarp::sig::Vector biggestAxis;
    yarp::sig::Vector vx,vy,vz;
    boundingBox.getAxis(vx,vy,vz);

    double normx=norm(vx);
    double normy=norm(vy);
    double normz=norm(vz);

    int indB;
    if (normx>normy && normx>normz)
    {
        biggestAxis=vx;
        indB=0;
    }
    else if (normy>normx && normy>normz)
    {
        biggestAxis=vy;
        indB=1;
    }
    else
    {
        biggestAxis=vz;
        indB=2;
    }

    biggestAxis/=norm(biggestAxis);
    return biggestAxis;
}

/************************************************************************/
string PowerGrasp::chooseBestPointAndOrientation(int &winnerIndex, yarp::sig::Matrix &designedOrientation)
{
    yarp::sig::Vector center(3), eePos(3);
    yarp::sig::Vector pyRight(3), pxRight(3), pyLeft(3), pxLeft(3);
    yarp::sig::Vector pointNormal(3), normalRight(3), normalLeft(3);
    yarp::sig::Vector tmpRight(3), tmpLeft(3);
    yarp::sig::Vector xhandaxisRight(3), xhandaxisLeft(3);
    Matrix orientationRight, orientationLeft;
    Matrix tmpOrientationRight, tmpOrientationLeft;
    double rightMan,leftMan;
    double bestRightMan=0.0;
    double bestLeftMan=0.0;
    int rightIndex=-1;
    int leftIndex=-1;

    yarp::sig::Vector x(3);
    x[0]=-1.0;
    x[1]=0.0;
    x[2]=0.0;
 
    string hand=NO_HAND;

    int indB;
    yarp::sig::Vector biggestAxis=findBiggestAxis(indB);
   
    if (!rightDisabled)
    {
        orientationThreadRight->reset();
        if (areCmdPort.getOutputCount()==0)
            orientationThreadRight->preAskForPose();
    }
    if (!leftDisabled)
    {
        orientationThreadLeft->reset();
        if (areCmdPort.getOutputCount()==0)
            orientationThreadLeft->preAskForPose();
    }
    for (int i=0; i<rankScores.size(); i++)
    {
        pxRight=0.0;
        
        pointNormal=vectorFromNormal(rankIndices[i]);
        normalRight=-1.0*pointNormal;
        normalLeft=pointNormal;

        tmpRight=0.0;
        tmpRight=cross(x,normalRight);
        pxRight=cross(normalRight,tmpRight);
        pxRight=pxRight/norm(pxRight);

        tmpLeft=0.0;
        tmpLeft=cross(x,normalLeft);
        pxLeft=cross(normalLeft,tmpLeft);
        pxLeft=pxLeft/norm(pxLeft);
        
        pyRight=cross(normalRight,pxRight);
        pyRight=pyRight/norm(pyRight);

        pyLeft=cross(normalLeft,pxLeft);
        pyLeft=pyLeft/norm(pyLeft);

        center=vectorFromCloud(rankIndices[i])+pointNormal;
        eePos=vectorFromCloud(rankIndices[i]);

        yarp::sig::Vector tmpBiggest=biggestAxis;

        if (!rightBlocked && !blockRightTmp && !rightDisabled)
        {
            orientationThreadRight->setInfo(eePos,pxRight,pyRight,normalRight,center,tmpBiggest);
            orientationThreadRight->resume();
        }
        if (!leftBlocked && !blockLeftTmp && !leftDisabled)
        {
            orientationThreadLeft->setInfo(eePos,pxLeft,pyLeft,normalLeft,center,tmpBiggest);
            orientationThreadLeft->resume();
        }

        if (!rightDisabled)
        {
            while (!orientationThreadRight->checkDone())
            {
                Time::delay(0.01);
            }
        }

        if (!leftDisabled)
        {
            while (!orientationThreadLeft->checkDone())
            {
                Time::delay(0.01);
            }
        }

        if (!rightBlocked && !blockRightTmp && !rightDisabled)
        {
            orientationThreadRight->getBestManip(rightMan,tmpOrientationRight);
            if (rightMan>bestRightMan)
            {
                bestRightMan=rightMan;
                orientationRight=tmpOrientationRight;
                rightIndex=i;
            }
        }

        if (!leftBlocked && !blockLeftTmp && !leftDisabled)
        {
            orientationThreadLeft->getBestManip(leftMan,tmpOrientationLeft);
            if (leftMan>bestLeftMan)
            {
                bestLeftMan=leftMan;
                orientationLeft=tmpOrientationLeft;
                leftIndex=i;
            }
        }
    }

    bool noResultR=true;
    bool noResultL=true;
    if (!rightBlocked && !blockRightTmp && !rightDisabled)
    {
        if (areCmdPort.getOutputCount()==0)
            orientationThreadRight->postAskForPose();
        noResultR=orientationThreadRight->getResult();
    }
    if (!leftBlocked && !blockLeftTmp && !leftDisabled)
    {
        if (areCmdPort.getOutputCount()==0)
            orientationThreadLeft->postAskForPose();
        noResultL=orientationThreadLeft->getResult();
    }

    if (noResultR && noResultL)
    {
        noResult=true;
        return hand;
    }
    if (bestLeftMan==0.0 && bestRightMan==0.0)
    {
        tooFar=true;
        return hand;
    }

    if (rightBlocked || blockRightTmp || bestLeftMan>bestRightMan)
    {
        winnerIndex=rankIndices[leftIndex];
        designedOrientation=orientationLeft;
        hand=LEFT_HAND;
    }

    if (leftBlocked || blockLeftTmp || bestRightMan>bestLeftMan)
    {
        winnerIndex=rankIndices[rightIndex];
        designedOrientation=orientationRight;
        hand=RIGHT_HAND;
    }

    return hand;
}

/************************************************************************/
void PowerGrasp::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    cloud->clear();
    cloudxyz->clear();

    for (size_t j = 0; j < cloud_in->points.size (); ++j)
    {
        PointXYZRGB pointrgb=cloud_in->points[j];
        pcl::PointXYZ point;
        point.x=pointrgb.x;
        point.y=pointrgb.y;
        point.z=pointrgb.z;
        cloudxyz->push_back(point);
        cloud->push_back(pointrgb);
    }
}

/************************************************************************/
double PowerGrasp::getPeriod()
{
    return 0.1;
}

/************************************************************************/
double PowerGrasp::scoreFunction(const int index)
{
    double score=0.0;
    
    yarp::sig::Vector point=vectorFromCloud(index);
    yarp::sig::Vector center=boundingBox.getCenter();

    if (point[2]<center[2])
        return 0.0;

    yarp::sig::Vector normal=vectorFromNormal(index);
    float curvature=normals->at(index).curvature;

    yarp::sig::Vector x,y,z;
    boundingBox.getAxis(x,y,z);
    x/=norm(x); y/=norm(y); z/=norm(z);

    yarp::sig::Vector dim=boundingBox.getDim();
    double minDim;
    if (dim[0]<dim[1])
        minDim=dim[0];
    else
        minDim=dim[1];
    if (dim[2]<minDim)
        minDim=dim[2];

    int count=0;
    for (int i=0; i<boundingBox.getCorners().size(); i++)
    {
        iCub::data3D::PointXYZ p=boundingBox.getCorners().at(i);
        yarp::sig::Vector tmp(3);
        tmp[0]=p.x;
        tmp[1]=p.y;
        tmp[2]=p.z;
        if (norm(tmp-point)>minDim/2)
            count++;
    }
    
    if (count==8)
        score+=1.0;

    if (train)
        return exp(-(fabs(curvature-currentCurvature)/maxCurvature))/2;

    if (testWithLearning)
    {
        Vector in(1,scalerIn.transform(curvature));
        Prediction prediction=machine.predict(in);
        Vector out=prediction.getPrediction();
        score+=scalerOut.unTransform(out[0]);
    }
    else
        score+=exp(-(fabs(curvature-currentCurvature)/maxCurvature))/2;

    if (currentModality==MODALITY_TOP)
    {
        if (point[2]-center[2]>0.0)
            return score+fabs(point[2]-center[2])/dimz;
        else
            return score;
    }
    else if (currentModality==MODALITY_RIGHT)
    {
        if (point[1]-center[1]>0.0)
        {
            yarp::sig::Vector tmp=center;
            tmp[1]=center[1]+(dimy/2);
            return score+exp(-(norm(point-tmp)/dimy))/2;
        }
        else
            return score;
    }
    else if (currentModality==MODALITY_LEFT)
    {
        if (point[1]-center[1]<0.0)
        {
            yarp::sig::Vector tmp=center;
            tmp[1]=center[1]-(dimy/2);
            return score+exp(-(norm(point-tmp)/dimy))/2;
        }
        else
            return score;
    }
    else
    {
        yarp::sig::Vector tmp(3);
        tmp=center;
        tmp[2]=center[2]+(dimz/2);

        return score+=exp(-(norm(point-tmp)/dimz))/2;
    }
}

/************************************************************************/
void PowerGrasp::insertElement(const double score, const int index)
{
    if (rankScores.size()==0)
    {
        rankScores.push_back(score);
        rankIndices.push_back(index);     
    }
    else if (rankScores.size()<numberOfBestPoints)
    {
        bool assigned=false;
        std::vector<int>::iterator itind=rankIndices.begin();
        for (std::vector<double>::iterator itsc = rankScores.begin(); itsc!=rankScores.end(); itsc++)
        {
            if (*itsc<score)
            {
                rankScores.insert(itsc,score);
                rankIndices.insert(itind,index);
                assigned=true;
                break;
            }
            itind++;
        }
        if (!assigned)
        {
            rankScores.push_back(score);
            rankIndices.push_back(index);
        }
    }
    else
    {
        if (rankScores[rankScores.size()-1]>score)
        {
            return;
        }
        else if (rankScores[0]<score)
        {
            std::vector<double>::iterator itsc=rankScores.begin();
            std::vector<int>::iterator itind=rankIndices.begin();
            rankScores.insert(itsc,score);
            rankIndices.insert(itind,index);
            rankScores.pop_back();
            rankIndices.pop_back();
        }
        else
        {
            std::vector<int>::iterator itind=rankIndices.begin();
            for (std::vector<double>::iterator itsc = rankScores.begin(); itsc!=rankScores.end(); itsc++)
            {
                if (*itsc<score)
                {
                    rankScores.insert(itsc,score);
                    rankIndices.insert(itind,index);
                    rankScores.pop_back();
                    rankIndices.pop_back();
                    break;
                }
                itind++;
            }
        }
    }
}

/************************************************************************/
bool PowerGrasp::normalPointingOut(const int index, const yarp::sig::Vector &center)
{
    yarp::sig::Vector p=vectorFromCloud(index);
    yarp::sig::Vector n=vectorFromNormal(index);

    yarp::sig::Vector fromCenter=(center-p)/norm(center-p);
    n=n/norm(n);
    
    return (dot(n,fromCenter)<0);
}

/************************************************************************/
void PowerGrasp::computeDim()
{
    maxy=-1.0;
    maxz=-1.0;
    double miny=1.0;
    double minz=1.0;
	for (int i=0; i<cloudxyz->size(); i++)
	{
	    pcl::PointXYZ point=cloudxyz->at(i);
            
        if (point.y>maxy)
            maxy=point.y;
        if (point.z>maxz)
            maxz=point.z;
        if (point.y<miny)
            miny=point.y;
        if (point.z<minz)
            minz=point.z;
    }

    printf("maxz %g minz %g max-min %g\n", maxz, minz, maxz-minz);
    dimz=maxz-minz;
    dimy=maxy-miny;
}


