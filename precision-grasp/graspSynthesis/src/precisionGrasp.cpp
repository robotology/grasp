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

#include "precisionGrasp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;
using namespace iCub::data3D;
using namespace iCub::grasp;
using namespace pcl::io;
using namespace pcl;

/************************************************************************/
PrecisionGrasp::PrecisionGrasp() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>), 
    cloudxyz(new pcl::PointCloud<pcl::PointXYZ>),
    normals (new pcl::PointCloud <pcl::Normal>)
{
    path="";

    dont=false;
    grasped=false;
    visualize=true;
    straight=false;
    rightBlocked=false;
    leftBlocked=false;
    fromFileFinished=false;
    filterCloud=true;
    clicked=false;
    writeCloud=true;
    nFile=0;
    counter=0;
    bestCost=1e20;
    bestManipulability=0.0;
    offsetL.resize(3,0.0);
    offsetR.resize(3,0.0);
    home_p_l.resize(3,0.0); home_p_l[0]=-0.27; home_p_l[2]=0.1;
    home_p_r.resize(3,0.0); home_p_r[0]=-0.27; home_p_r[2]=0.1;
    home_o_l.resize(4,0.0);
    home_o_r.resize(4,0.0);

    current_state=STATE_WAIT;
    visualizationThread=new VisualizationThread(data);
    psoThreadFitness1=new PsoThread();
    psoThreadFitness2=new PsoThread();
    psoThreadFitness3=new PsoThread();
    psoThreadFitness4=new PsoThread();
}

/************************************************************************/
bool PrecisionGrasp::configure(ResourceFinder &rf)
{
    radiusSearch=rf.check("radiusSearch",Value(0.045)).asDouble();
    handSize=rf.check("limit_finger_max",Value(0.08)).asDouble();
    double limit_finger_min=rf.check("limit_finger_min",Value(0.02)).asDouble();
    string name=rf.check("name",Value("precision-grasp")).asString().c_str();
    string useFile=rf.check("fromFile",Value("false")).asString().c_str();
    path=rf.find("path").asString().c_str();
    sampling=rf.check("sampling",Value(0.01f)).asDouble();
    alpha=rf.check("alpha",Value(0.9)).asDouble();
    robot=rf.check("robot",Value("icub")).asString();
    double phi_p=rf.check("phi_p",Value(0.001)).asDouble();
    double phi_g=rf.check("phi_g",Value(0.001)).asDouble();
    int iterations=rf.check("iterations",Value(2000)).asInt();
    double hand_area=rf.check("hand_area",Value(0.00225)).asDouble();
    outputFile=rf.find("outputFile").asString().c_str();
    posx=rf.check("x",Value(0)).asInt();
    posy=rf.check("y",Value(0)).asInt();
    visualizationThread->setPosition(posx, posy);
    sizex=rf.check("w",Value(320)).asInt();
    sizey=rf.check("h",Value(240)).asInt();
    visualizationThread->setSize(sizex, sizey);
    
    if (!openDevices())
        return false;

    Property psoProperty;
    psoProperty.put("particles",20);
    psoProperty.put("omega",1.0);
    psoProperty.put("phi_p",phi_p);
    psoProperty.put("phi_g",phi_g);
    psoProperty.put("iterations",iterations);
    psoProperty.put("limit_finger_max",handSize);
    psoProperty.put("limit_finger_min",limit_finger_min);
    psoProperty.put("hand_area",hand_area);
    psoProperty.put("dimension",9);

    psoThreadFitness1->open(psoProperty);
    psoThreadFitness2->open(psoProperty);
    psoThreadFitness3->open(psoProperty);
    psoThreadFitness4->open(psoProperty);

    fromFile=(useFile=="true");
    
    if (fromFile)
    {
        path=rf.find("path").asString().c_str();
        if (path=="")
            return false;
    }

    //Ports opening
    rpc.open(("/"+name+"/rpc").c_str());
    attach(rpc);

    depth2kin.open(("/"+name+"/depth2kin:o").c_str());

    toMatlab.open(("/"+name+"/matlab").c_str());

    if (!ikPort1r.open(("/"+name+"/ik1r:o").c_str()))
        return false;

    if (!ikPort2r.open(("/"+name+"/ik2r:o").c_str()))
        return false;

    if (!ikPort3r.open(("/"+name+"/ik3r:o").c_str()))
        return false;

    if (!ikPort4r.open(("/"+name+"/ik4r:o").c_str()))
        return false;

    if (!ikPort1l.open(("/"+name+"/ik1l:o").c_str()))
        return false;

    if (!ikPort2l.open(("/"+name+"/ik2l:o").c_str()))
        return false;

    if (!ikPort3l.open(("/"+name+"/ik3l:o").c_str()))
        return false;

    if (!ikPort4l.open(("/"+name+"/ik4l:o").c_str()))
        return false;

    if (!meshPort.open(("/"+name+"/mesh:i").c_str()))
        return false;

    if (!reconstructionPort.open(("/"+name+"/reconstruction").c_str()))
        return false;

    if (!areCmdPort.open(("/"+name+"/are/cmd:o").c_str()))
        return false;

    if (fromFile)
    {
        Network::connect(ikPort1r.getName().c_str(), "/handIKModule1/right/rpc");
        Network::connect(ikPort1l.getName().c_str(), "/handIKModule1/left/rpc");
        Network::connect(ikPort2r.getName().c_str(), "/handIKModule2/right/rpc");
        Network::connect(ikPort2l.getName().c_str(), "/handIKModule2/left/rpc");
        Network::connect(ikPort3r.getName().c_str(), "/handIKModule3/right/rpc");
        Network::connect(ikPort3l.getName().c_str(), "/handIKModule3/left/rpc");
        Network::connect(ikPort4r.getName().c_str(), "/handIKModule4/right/rpc");
        Network::connect(ikPort4l.getName().c_str(), "/handIKModule4/left/rpc");
    }

    return true;
}

/************************************************************************/
bool PrecisionGrasp::openDevices()
{
    Property optCtrlRight;
    optCtrlRight.put("device","cartesiancontrollerclient");
    optCtrlRight.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optCtrlRight.put("local","/orientation/right/cartesianRight");

    if (!dCtrlRight.open(optCtrlRight))
    {
        fprintf(stdout, "Right Cartesian Interface is not open\n");
        return false;
    }

    dCtrlRight.view(iCtrlRight);
    iCtrlRight->storeContext(&context_in_right);
    yarp::sig::Vector dof;
    iCtrlRight->getDOF(dof);
    yarp::sig::Vector newDof=dof;
    newDof[0]=1.0;
    newDof[2]=1.0;

    iCtrlRight->setDOF(newDof,dof);
    iCtrlRight->storeContext(&context_right);
    iCtrlRight->restoreContext(context_in_right);

    Property optCtrlLeft;
    optCtrlLeft.put("device","cartesiancontrollerclient");
    optCtrlLeft.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
    optCtrlLeft.put("local","/orientation/left/cartesianLeft");

    if (!dCtrlLeft.open(optCtrlLeft))
    {
        fprintf(stdout, "Left Cartesian Interface is not open\n");
        return false;
    }

    dCtrlLeft.view(iCtrlLeft);
    iCtrlLeft->storeContext(&context_in_left);
    dof.clear();
    iCtrlLeft->getDOF(dof);
    newDof=dof;
    newDof[0]=1.0;
    newDof[2]=1.0;

    iCtrlLeft->setDOF(newDof,dof);
    iCtrlLeft->storeContext(&context_left);
    iCtrlLeft->restoreContext(context_in_left);

    Property optArmRight,optArmLeft;
    Property optTorso;

    string remoteArmNameRight="/"+robot+"/right_arm";
    optArmRight.put("device", "remote_controlboard");
    optArmRight.put("remote",remoteArmNameRight.c_str());
    optArmRight.put("local","/localArm/right");

    string remoteArmNameLeft="/"+robot+"/left_arm";
    optArmLeft.put("device", "remote_controlboard");
    optArmLeft.put("remote",remoteArmNameLeft.c_str());
    optArmLeft.put("local","/localArm/left");

    string remoteTorsoName="/"+robot+"/torso";
    optTorso.put("device", "remote_controlboard");
    optTorso.put("remote",remoteTorsoName.c_str());
    optTorso.put("local","/localTorso");

    robotTorso.open(optTorso);
    robotArmRight.open(optArmRight);
    robotArmLeft.open(optArmLeft);

    if (!robotTorso.isValid() || !robotArmRight.isValid() || !robotArmLeft.isValid())
    {
        fprintf(stdout, "Device not available\n");
        return false;
    }

    robotArmRight.view(limArmRight);
    robotArmRight.view(posRight);
    robotArmRight.view(encRight);
    robotArmLeft.view(limArmLeft);
    robotArmLeft.view(posLeft);
    robotArmLeft.view(encLeft);
    robotTorso.view(limTorso);

    armRight=new iCubArm("right");
    armLeft=new iCubArm("left");

    chainRight=armRight->asChain();
    chainLeft=armLeft->asChain();

    chainRight->releaseLink(0);
    chainRight->releaseLink(1);
    chainRight->releaseLink(2);

    chainLeft->releaseLink(0);
    chainLeft->releaseLink(1);
    chainLeft->releaseLink(2);

    deque<IControlLimits*> limRight;
    limRight.push_back(limTorso);
    limRight.push_back(limArmRight);
    armRight->alignJointsBounds(limRight);

    armRight->setAllConstraints(false);

    deque<IControlLimits*> limLeft;
    limLeft.push_back(limTorso);
    limLeft.push_back(limArmLeft);
    armLeft->alignJointsBounds(limLeft);

    armLeft->setAllConstraints(false);

    thetaMinRight.resize(10,0.0);
    thetaMaxRight.resize(10,0.0);
    for (unsigned int i=0; i< chainRight->getDOF(); i++)
    {
       thetaMinRight[i]=(*chainRight)(i).getMin();
       thetaMaxRight[i]=(*chainRight)(i).getMax();
    }

    thetaMinLeft.resize(10,0.0);
    thetaMaxLeft.resize(10,0.0);
    for (unsigned int i=0; i< chainLeft->getDOF(); i++)
    {
       thetaMinLeft[i]=(*chainLeft)(i).getMin();
       thetaMaxLeft[i]=(*chainLeft)(i).getMax();
    }

    return true;
}

/************************************************************************/
string PrecisionGrasp::extractData(const yarp::os::Bottle &data, const int t)
{
    Bottle &b=const_cast<Bottle&>(data);
    string hand=b.find("hand").asString().c_str();
    double cost=b.find("cost").asDouble();
    Bottle *eeB=b.find("ee").asList();
    fillVectorFromBottle(eeB,ee_tmp);
    Bottle *oright=b.find("or").asList();
    fillVectorFromBottle(oright,axis_angle_tmp);
    Bottle *j=b.find("joints").asList();
    fillVectorFromBottle(j,joints_tmp);
    Bottle* combination=b.find("combination").asList();
    fillVectorFromBottle(combination,combination_tmp);

    yarp::dev::ICartesianControl *iCtrl;
    iCub::iKin::iCubArm *arm;
    yarp::sig::Vector *thetaMin=&thetaMinRight;
    yarp::sig::Vector *thetaMax=&thetaMaxRight;
    q.resize(10,0.0);

    if (hand=="right")
    {
        iCtrl=iCtrlRight;
        arm=armRight;
        thetaMin=&thetaMinRight;
        thetaMax=&thetaMaxRight;
    }
    else
    {
        iCtrl=iCtrlLeft;
        arm=armLeft;
        thetaMin=&thetaMinLeft;
        thetaMax=&thetaMaxLeft;
    }
    
    double manipulability=0.0;

    iCtrl->askForPose(ee_tmp,axis_angle_tmp,xdhat,odhat,q);
    
    q=q*M_PI/180.0;
    arm->setAng(q);

    yarp::sig::Vector od(3);
    od[0]=axis_angle_tmp[0]*axis_angle_tmp[3];
    od[1]=axis_angle_tmp[1]*axis_angle_tmp[3];
    od[2]=axis_angle_tmp[2]*axis_angle_tmp[3];

    yarp::sig::Vector odhattmp(3);
    odhattmp[0]=odhat[0]*odhat[3];
    odhattmp[1]=odhat[1]*odhat[3];
    odhattmp[2]=odhat[2]*odhat[3];

    double xdist=norm(ee_tmp-xdhat);
    double odist=norm(od-odhattmp);

    if (xdist>0.01 && odist>0.1)
        manipulability=0.0;
    else
    {
        Matrix jacobian=arm->GeoJacobian();
        Matrix mulJac=jacobian*(jacobian.transposed());

        manipulability=sqrt(det(mulJac));
        
        double limits=0.0;
        for (unsigned int k=0; k<thetaMin->size(); k++)
        {
            limits+=(q[k]-((*thetaMin)[k]))*((*thetaMax)[k]-q[k])/(((*thetaMax)[k]-(*thetaMin)[k])*((*thetaMax)[k]-(*thetaMin)[k]));
        }
        
        manipulability*=(1-exp(-limits));
    }

    printf("\n\nmanipulability %g\n", manipulability);
    printf("cost %g\n\n", cost);

    double toll=0.2;
    if (cost<bestCost+toll && manipulability>(bestManipulability-toll))
    {
        string tag_0=b.get(0).asString().c_str();
        if (tag_0=="IK1")
            winner_ov_cones=ov_cones1;
        else if (tag_0=="IK2")
            winner_ov_cones=ov_cones2;
        else if (tag_0=="IK3")
            winner_ov_cones=ov_cones3;
        else
            winner_ov_cones=ov_cones4;
        winner_joints=joints_tmp;
        winner_ee=ee_tmp;
        winner_axis=axis_angle_tmp;
        winner_combination=combination_tmp;
        winner_hand=hand;
        winner_triplet=t;
        winner_xdhat=xdhat;
        winner_odhat=odhat;
        bestCost=cost;
        bestManipulability=manipulability;
    }

    return hand;
}

/************************************************************************/
void PrecisionGrasp::fillVectorFromBottle(const yarp::os::Bottle* b, yarp::sig::Vector &v)
{
    v.resize(b->size());
    for (int i=0; i<b->size(); i++)
        v[i]=b->get(i).asDouble();
}

/************************************************************************/
bool PrecisionGrasp::interruptModule()
{
    eventRpc.signal();
    Bottle &out=toMatlab.prepare();
    out.clear();
    out.addString("quit");
    toMatlab.write();
    toMatlab.interrupt();
    ikPort1r.interrupt();
    ikPort2r.interrupt();
    ikPort3r.interrupt();
    ikPort4r.interrupt();
    ikPort1l.interrupt();
    ikPort2l.interrupt();
    ikPort3l.interrupt();
    ikPort4l.interrupt();
    depth2kin.interrupt();
    areCmdPort.interrupt();
    reconstructionPort.interrupt();
    meshPort.interrupt();
    rpc.interrupt();

    if (psoThreadFitness1->isRunning())
        psoThreadFitness1->stop();

    if (psoThreadFitness2->isRunning())
        psoThreadFitness2->stop();

   if (psoThreadFitness3->isRunning())
        psoThreadFitness3->stop();

   if (psoThreadFitness4->isRunning())
        psoThreadFitness4->stop();

    return true;
}

/************************************************************************/
bool PrecisionGrasp::close()
{
    ikPort1r.close();
    ikPort2r.close();
    ikPort3r.close();
    ikPort4r.close();
    ikPort1l.close();
    ikPort2l.close();
    ikPort3l.close();
    ikPort4l.close();
    areCmdPort.close();
    depth2kin.close();
    reconstructionPort.close();
    meshPort.close();
    rpc.close(); 
    toMatlab.close();

    if (graspFileTrain.is_open())
        graspFileTrain.close();

    if (visualizationThread->isRunning())
        visualizationThread->stop();
    delete visualizationThread;

    if (psoThreadFitness1->isRunning())
        psoThreadFitness1->stop();
    delete psoThreadFitness1;

    if (psoThreadFitness2->isRunning())
        psoThreadFitness2->stop();
    delete psoThreadFitness2;

    if (psoThreadFitness3->isRunning())
        psoThreadFitness3->stop();
    delete psoThreadFitness3;

    if (psoThreadFitness4->isRunning())
        psoThreadFitness4->stop();
    delete psoThreadFitness4;

    if (dCtrlRight.isValid())
    {
        iCtrlRight->restoreContext(context_in_right);
        dCtrlRight.close();
    }
    if (dCtrlLeft.isValid())
    {
        iCtrlLeft->restoreContext(context_in_left);
        dCtrlLeft.close();
    }
    if (robotArmRight.isValid())
        robotArmRight.close();
    if (robotArmLeft.isValid())
        robotArmLeft.close();
    if (robotTorso.isValid())
        robotTorso.close();

    return true;
}

/************************************************************************/
void PrecisionGrasp::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered, bool second)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (cloud_in->size()/2);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_in_filtered);
}

/************************************************************************/
void PrecisionGrasp::write(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const string &fileName)
{
    ofstream myfile;
    myfile.open (fileName.c_str());
    /*myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << cloud_in->size() << "\n";
    myfile << "property float x\n";
    myfile << "property float y\n";
    myfile << "property float z\n";
    myfile << "property uchar diffuse_red\n";
    myfile << "property uchar diffuse_green\n";
    myfile << "property uchar diffuse_blue\n";
    myfile << "end_header\n";*/
    
    for (int i=0; i<cloud_in->size(); i++)
    {
        int r=cloud_in->at(i).r;
        int g=cloud_in->at(i).g;
        int b=cloud_in->at(i).b;
        myfile << cloud_in->at(i).x << " " << cloud_in->at(i).y << " " << cloud_in->at(i).z << " " << r << " " << g << " " << b << "\n";
    }
    myfile.close();
}

/************************************************************************/
void PrecisionGrasp::write(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud <pcl::Normal>::Ptr n, const string &fileName)
{
    ofstream myfile;
    myfile.open (fileName.c_str());
    for (int i=0; i<cloud_in->size(); i++)
    {
        myfile << cloud_in->at(i).x << " " << cloud_in->at(i).y << " " << cloud_in->at(i).z << " " << n->at(i).normal_x << " " << n->at(i).normal_y << " " << n->at(i).normal_z << "\n";
    }
    myfile.close();
}

/************************************************************************/
void PrecisionGrasp::fromSurfaceMesh (const SurfaceMeshWithBoundingBox& msg)
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
bool PrecisionGrasp::fillCloudFromFile()
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
        {
            cout << "cannot read file \n";
            return false;
        }

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
void PrecisionGrasp::sampleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud <pcl::Normal>::Ptr n)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (128.0f);
    octree.setInputCloud(cloudxyz);
    octree.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

    if (cloudxyz->size()<100)
        sampling=0.01;

    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
    voxfilter.setInputCloud (cloudxyz);
    voxfilter.setLeafSize (sampling,sampling,sampling);
    voxfilter.filter(*cloud_downsampled);

    yarp::sig::Vector vx,vy,vz; boundingBox.getAxis(vx,vy,vz);
    double zdim=getZDim(vx,vy,vz);
    double scale=10.0;
    double threshold=center[2]-(zdim/scale);
    if (zdim<(handSize/2))
        threshold=center[2]-zdim;
    /*double numberOfPoints=percentage*((double)cloud->size());
    int factorI=(int)((double)cloud->size()/numberOfPoints);*/
   
    for (int i=0; i<cloud_downsampled->size(); i++)
    {
        int k=1;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;

        int id;
        if (octree.nearestKSearch (cloud_downsampled->at(i), k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            id=pointIdxNKNSearch[0];
            if (normalPointingOut(normals->at(id),cloudxyz->at(id)) && cloudxyz->points.at(id).z>threshold)
            {
                c->push_back(cloudxyz->at(id));
                n->push_back(normals->at(id));
            }
        }
    }
    printf("points %d\n", c->size());
}

/************************************************************************/
bool PrecisionGrasp::updateModule()
{
    if ((fromFile && !fromFileFinished) || (current_state==STATE_ESTIMATE))
    {
        double totTime=Time::now();
        iCtrlRight->deleteContext(current_context_right);
        iCtrlRight->storeContext(&current_context_right);
        iCtrlRight->restoreContext(context_right);
        iCtrlLeft->deleteContext(current_context_left);
        iCtrlLeft->storeContext(&current_context_left);
        iCtrlLeft->restoreContext(context_left);
        if (fromFile)
        {
            if (!fillCloudFromFile())
                return false;
            boundingBox=MinimumBoundingBox::getMinimumBoundingBox(cloud);
        }    
        else if (current_state==STATE_ESTIMATE)
        {
            SurfaceMeshWithBoundingBox *receivedMesh=meshPort.read(false);
            if (receivedMesh!=NULL)
            {
                if (receivedMesh->mesh.points.size()==0)
                {
                    current_state=STATE_WAIT;
                    return true;
                }
                fromSurfaceMesh(*receivedMesh);
            }
            else
                return true;
        }

        if (writeCloud)
            write(cloud,outputFile);

        center=boundingBox.getCenter();
        dim=boundingBox.getDim();
        rotation=boundingBox.getOrientation();

        associateDim();

        //Normal Estimation
        double timeNormals=Time::now();
        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setRadiusSearch (radiusSearch);
        normal_estimator.setInputCloud (cloudxyz);
        normal_estimator.compute (*normals);
        printf("Time for normal estimation %g\n", Time::now()-timeNormals);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_toEvaluate (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals_toEvaluate (new pcl::PointCloud <pcl::Normal>);

        sampleClouds(cloud_toEvaluate,normals_toEvaluate);

        //write(cloud_toEvaluate,normals_toEvaluate,"C:/Users/Utente/Desktop/points.txt");
        //readData(cloud_toEvaluate,normals_toEvaluate);

        if (visualize)
        {
            if (!fromFile || (fromFile && !fromFileFinished))
            {
                data.cloud=*cloud;
                data.normals=*normals;
                data.sampled_cloud=*cloud_toEvaluate;
                data.boundingBox=boundingBox;
                visualizationThread->start();
            }
        }

        bool done=false;
        //double alphaToUse=alpha;
        double alphaToUse=0.8;
        double t=Time::now();
        psoThreadFitness1->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,0);
        psoThreadFitness2->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,1);
        psoThreadFitness3->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,2);
        psoThreadFitness4->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,3);
        while(!done)
        {
            while(!psoThreadFitness1->checkDone() || !psoThreadFitness2->checkDone() || !psoThreadFitness3->checkDone() || !psoThreadFitness4->checkDone())
                Time::delay(0.01);

            Bottle req; req.clear();
            if (psoThreadFitness1->isSuccessful())
            {
                done=true;
                ContactPoints triplet1=psoThreadFitness1->getBestTriplet();
                contacts_r1.clear();
                contacts_r1.push_back(triplet1.c1); contacts_r1.push_back(triplet1.c2); contacts_r1.push_back(triplet1.c3);
                normals_r1.clear();
                normals_r1.push_back(triplet1.n1); normals_r1.push_back(triplet1.n2); normals_r1.push_back(triplet1.n3);

                ov_cones1=triplet1.ov_cones;

                req=prepareData(triplet1,1);
                if (ikPort1r.getOutputCount()>0)
                {
                    ikPort1r.write(req);
                    counter+=1;
                 }
                if (ikPort1l.getOutputCount()>0)
                {
                    ikPort1l.write(req);
                    counter+=1;
                 }
            }
            if (psoThreadFitness2->isSuccessful())
            {
                done=true;
                ContactPoints triplet2=psoThreadFitness2->getBestTriplet();
                contacts_r2.clear();
                contacts_r2.push_back(triplet2.c1); contacts_r2.push_back(triplet2.c2); contacts_r2.push_back(triplet2.c3);
                normals_r2.clear();
                normals_r2.push_back(triplet2.n1); normals_r2.push_back(triplet2.n2); normals_r2.push_back(triplet2.n3);

                ov_cones2=triplet2.ov_cones;

                req.clear();
                req=prepareData(triplet2,2);
                if (ikPort2r.getOutputCount()>0)
                {
                    ikPort2r.write(req);
                    counter+=1;
                }
                if (ikPort2l.getOutputCount()>0)
                {
                    ikPort2l.write(req);
                    counter+=1;
                }
            }
            if (psoThreadFitness3->isSuccessful())
            {
                done=true;
                ContactPoints triplet3=psoThreadFitness3->getBestTriplet();
                contacts_r3.clear();
                contacts_r3.push_back(triplet3.c1); contacts_r3.push_back(triplet3.c2); contacts_r3.push_back(triplet3.c3);
                normals_r3.clear();
                normals_r3.push_back(triplet3.n1); normals_r3.push_back(triplet3.n2); normals_r3.push_back(triplet3.n3);

                ov_cones3=triplet3.ov_cones;

                req.clear();
                req=prepareData(triplet3,3);
                if (ikPort3r.getOutputCount()>0)
                {
                    ikPort3r.write(req);
                    counter+=1;
                }
                if (ikPort3l.getOutputCount()>0)
                {
                    ikPort3l.write(req);
                    counter+=1;
                }
            }
            if (psoThreadFitness4->isSuccessful())
            {
                done=true;
                ContactPoints triplet4=psoThreadFitness4->getBestTriplet();

                contacts_r4.clear();
                contacts_r4.push_back(triplet4.c1); contacts_r4.push_back(triplet4.c2); contacts_r4.push_back(triplet4.c3);
                normals_r4.clear();
                normals_r4.push_back(triplet4.n1); normals_r4.push_back(triplet4.n2); normals_r4.push_back(triplet4.n3);

                ov_cones4=triplet4.ov_cones;

                req.clear();
                req=prepareData(triplet4,4);
                if (ikPort4r.getOutputCount()>0)
                {
                    ikPort4r.write(req);
                    counter+=1;
                }
                if (ikPort4l.getOutputCount()>0)
                {
                    ikPort4l.write(req);
                    counter+=1;
                }
            }

            if ((!psoThreadFitness1->isSuccessful()) && (!psoThreadFitness2->isSuccessful()) && (!psoThreadFitness3->isSuccessful()) && (!psoThreadFitness4->isSuccessful()) && alphaToUse<=0.9)
            {
                alphaToUse+=0.1;
                psoThreadFitness1->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,0);
                psoThreadFitness2->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,1);
                psoThreadFitness3->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,2);
                psoThreadFitness4->setData(cloud_toEvaluate,normals_toEvaluate,alphaToUse,3);
            }
            else if ((!psoThreadFitness1->isSuccessful()) && (!psoThreadFitness2->isSuccessful()) && (!psoThreadFitness3->isSuccessful()) && (!psoThreadFitness4->isSuccessful()) && alphaToUse>0.9)
            {
                done=true;
                grasped=false;
                printf("Didn't find any triplet\n");
                eventRpc.signal();
                current_state=STATE_WAIT;
                if (fromFile)
                    fromFileFinished=true;
                return true;
            }
        }

        printf("Time %g\n", Time::now()-t);

        current_state=STATE_IK;

        if (!dont)
            dont=false;

        if (fromFile)
            fromFileFinished=true;
    }

    if (current_state==STATE_GRASP)
    {
        mutex.wait();
        askToGrasp();
        mutex.post();
        eventRpc.signal();
        current_state=STATE_WAIT;
    }
    
    return true;
}

/************************************************************************/
void PrecisionGrasp::associateDim()
{
    yarp::sig::Vector x,y,z;
    boundingBox.getAxis(x,y,z);

    yarp::sig::Vector dim_tmp(3,0.0);

    yarp::sig::Vector tmpx(3,0.0);
    tmpx[0]=1.0;

    yarp::sig::Vector tmpy(3,0.0);
    tmpy[1]=1.0;

    yarp::sig::Vector tmpz(3,0.0);
    tmpx[2]=1.0;

    double dotx1=abs(dot(x,tmpx)); double doty1=abs(dot(x,tmpy)); 
    double dotx2=abs(dot(y,tmpx)); double doty2=abs(dot(y,tmpy)); 
    double dotx3=abs(dot(z,tmpx)); double doty3=abs(dot(z,tmpy)); 

    if (dotx1<dotx2 && dotx1<dotx3)
    {
        dim_tmp[0]=dim[0];
        if (doty2<doty3)
        {
            dim_tmp[1]=dim[1];
            dim_tmp[2]=dim[2];
        }
        else if (doty2>doty3)
        {
            dim_tmp[2]=dim[1];
            dim_tmp[1]=dim[2];
        }
    }
    else if (dotx2<dotx1 && dotx2<dotx3)
    {
        dim_tmp[0]=dim[1];
        if (doty1<doty3)
        {
            dim_tmp[1]=dim[0];
            dim_tmp[2]=dim[2];
        }
        else if (doty1>doty3)
        {
            dim_tmp[1]=dim[2];
            dim_tmp[2]=dim[0];
        }
    }
    else 
    {
        dim_tmp[0]=dim[2];
        if (doty1<doty2)
        {
            dim_tmp[1]=dim[0];
            dim_tmp[2]=dim[1];
        }
        else if (doty1>doty2)
        {
            dim_tmp[1]=dim[1];
            dim_tmp[2]=dim[0];
        }
    }

    dim=dim_tmp;
}

/************************************************************************/
void PrecisionGrasp::readData(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud <pcl::Normal>::Ptr n)
{
    ifstream infile("C:\\Users\\Utente\\MATLABdata\\points\\points1.txt");
    double c1,c2,c3,n1,n2,n3;
    while(infile >> c1 >> c2 >> c3 >> n1 >> n2 >> n3)
    {
        pcl::PointXYZ p(c1,c2,c3);
        c->push_back(p);
        pcl::Normal normal(n1,n2,n3);
        n->push_back(normal);
    }
}

/************************************************************************/
double PrecisionGrasp::getZDim(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz)
{
    yarp::sig::Vector z(3,0.0); z[2]=1.0;

    double cx=abs(dot(vx,z));
    double cy=abs(dot(vy,z));
    double cz=abs(dot(vz,z));

    int index=-1;
    if (cx>cy && cx>cz)
        return norm(vx);
    else if (cy>cz)
        return norm(vy);
    else
        return norm(vz);
}

/************************************************************************/
yarp::os::Bottle PrecisionGrasp::prepareData(const iCub::grasp::ContactPoints &triplet, const int c)
{
    Bottle res;
    if (c==1)
        res.addString("IK1");
    else if (c==2)
        res.addString("IK2");
    else if (c==3)
        res.addString("IK3");
    else
        res.addString("IK4");
    Bottle &centerB=res.addList();
    centerB.addString("center");
    Bottle &center_coord=centerB.addList();
    for (int i=0; i<center.size(); i++)
        center_coord.addDouble(center[i]);
    Bottle &dimB=res.addList();
    dimB.addString("dim");
    Bottle &dim_coord=dimB.addList();
    for (int i=0; i<dim.size(); i++)
        dim_coord.addDouble(dim[i]);
    Bottle &c1=res.addList();
    c1.addString("c1");
    Bottle &c1_coord=c1.addList();
    for (int i=0; i<triplet.c1.size(); i++)
        c1_coord.addDouble(triplet.c1[i]);
    Bottle &c2=res.addList();
    c2.addString("c2");
    Bottle &c2_coord=c2.addList();
    for (int i=0; i<triplet.c2.size(); i++)
        c2_coord.addDouble(triplet.c2[i]);
    Bottle &c3=res.addList();
    c3.addString("c3");
    Bottle &c3_coord=c3.addList();
    for (int i=0; i<triplet.c3.size(); i++)
        c3_coord.addDouble(triplet.c3[i]);
    Bottle &n1=res.addList();
    n1.addString("n1");
    Bottle &n1_coord=n1.addList();
    for (int i=0; i<triplet.n1.size(); i++)
        n1_coord.addDouble(triplet.n1[i]);
    Bottle &n2=res.addList();
    n2.addString("n2");
    Bottle &n2_coord=n2.addList();
    for (int i=0; i<triplet.n2.size(); i++)
        n2_coord.addDouble(triplet.n2[i]);
    Bottle &n3=res.addList();
    n3.addString("n3");
    Bottle &n3_coord=n3.addList();
    for (int i=0; i<triplet.n3.size(); i++)
        n3_coord.addDouble(triplet.n3[i]);
    Bottle &rot=res.addList();
    rot.addString("rot");
    Bottle &rot_mat=rot.addList();
    for (int i=0; i<rotation.rows(); i++)
        for (int j=0; j<rotation.cols(); j++)
            rot_mat.addDouble(rotation(i,j));
    return res;
}

/************************************************************************/
void PrecisionGrasp::askToGrasp()
{
    Bottle b,reply;
    b.clear(); reply.clear();
    b.addString("getPoint");
    b.addString(winner_hand.c_str());
    b.addDouble(winner_ee[0]);
    b.addDouble(winner_ee[1]);
    b.addDouble(winner_ee[2]);

    depth2kin.write(b,reply);

    winner_ee[0]=reply.get(1).asDouble();
    winner_ee[1]=reply.get(2).asDouble();
    winner_ee[2]=reply.get(3).asDouble();
    
    if (winner_hand=="right")
    {
        printf("winner %s\n", winner_ee.toString().c_str());
        printf("offset %s\n", offsetR.toString().c_str());
        winner_ee+=offsetR;
    }
    else
    {
        printf("winner %s\n", winner_ee.toString().c_str());
        printf("offset %s\n", offsetL.toString().c_str());
        winner_ee+=offsetL;
    }

    reply.clear();
    Bottle cmd;
    cmd.addString("pgrasp");
    Bottle &point=cmd.addList();
    point.addDouble(winner_ee[0]);
    point.addDouble(winner_ee[1]);
    point.addDouble(winner_ee[2]);
    point.addDouble(winner_axis[0]);
    point.addDouble(winner_axis[1]);
    point.addDouble(winner_axis[2]);
    point.addDouble(winner_axis[3]);
    cmd.addString(winner_hand.c_str());

    Bottle &bJoints=cmd.addList();
    bJoints.addString("joints");
    Bottle &pos=bJoints.addList();

    yarp::sig::Vector joints(9);
    joints[0]=winner_joints[3]*180.0/M_PI;
    joints[1]=winner_joints[0]*180.0/M_PI;
    joints[2]=winner_joints[1]*180.0/M_PI;
    joints[3]=winner_joints[2]*180.0/M_PI;
    joints[4]=winner_joints[4]*180.0/M_PI;
    joints[5]=winner_joints[5]*180.0/M_PI;
    joints[6]=winner_joints[6]*180.0/M_PI;
    joints[7]=winner_joints[7]*180.0/M_PI;
    joints[8]=0.0;

    for (size_t i=0; i<joints.size(); i++)
        pos.addDouble(joints[i]);

    yarp::sig::Vector speed(joints.size(),40.0);

    Bottle &bVels=cmd.addList();
    bVels.addString("vels");
    Bottle &v=bVels.addList();
    for (size_t i=0; i<speed.size(); i++)
        v.addDouble(speed[i]);

    Bottle &bTols=cmd.addList();
    bTols.addString("tols");
    Bottle &t=bTols.addList();
    t.addDouble(20.0);
    for (int i=0; i<7; i++)
        t.addDouble(10.0);
    t.addDouble(20.0);

    Bottle &bThresh=cmd.addList();
    bThresh.addString("thres");
    Bottle &th=bThresh.addList();
    for (int i=0; i<5; i++)
        th.addDouble(120.0);

    Bottle &tmo=cmd.addList();
    tmo.addString("tmo");
    tmo.addDouble(2.0);

    if (areCmdPort.getOutputCount()>0)
    {
        areCmdPort.write(cmd,reply);
        if (reply.get(0).asString()=="ack")
            grasped=true;
    }
    readyToGrasp=false;
    /*ICartesianControl *iCtrl;
    IEncoders *encs;
    IPositionControl *pos;
    int curr_context;
    int context;
    yarp::sig::Vector offset;
    yarp::sig::Vector home_p;
    yarp::sig::Vector home_o;
    if (winner_hand=="right")
    {
        iCtrl=iCtrlRight;
        encs=encRight;
        pos=posRight;
        curr_context=current_context_right;
        context=context_right;
        offset=offsetR;
        home_p=home_p_r;
        home_o=home_o_r;
    }
    else
    {
        iCtrl=iCtrlLeft;
        encs=encLeft;
        pos=posLeft;
        curr_context=current_context_left;
        context=context_left;
        offset=offsetL;
        home_p=home_p_l;
        home_o=home_o_l;
    }

    iCtrl->deleteContext(curr_context);
    iCtrl->storeContext(&curr_context);
    iCtrl->restoreContext(context);
    winner_ee+=offset;
    iCtrl->goToPoseSync(winner_ee, winner_axis);
    iCtrl->waitMotionDone();

    yarp::sig::Vector target(16);
    encs->getEncoders(target.data());
    target[7]=winner_joints[3]*180.0/M_PI;
    target[8]=winner_joints[0]*180.0/M_PI;
    target[9]=winner_joints[1]*180.0/M_PI;
    target[10]=winner_joints[2]*180.0/M_PI;
    target[11]=winner_joints[4]*180.0/M_PI;
    target[12]=winner_joints[5]*180.0/M_PI;
    target[13]=winner_joints[6]*180.0/M_PI;
    target[14]=winner_joints[7]*180.0/M_PI;

    yarp::sig::Vector speed(target.size(),100.0);
    pos->positionMove(target.data());
    yarp::sig::Vector encoders(16);
    bool done=false;
    double t=Time::now();
    while (!done)
    {
        encoders=encs->getEncoders(encoders.data());
        if ((norm(encoders-target)<1e-01) || (Time::now()-t)>5.0)
            done=true;
    }

    Time::delay(1.5);

    winner_ee[2]+=0.1;
    iCtrl->goToPoseSync(home_p, home_o);
    iCtrl->waitMotionDone();

    iCtrl->restoreContext(curr_context);*/
}

/************************************************************************/
void PrecisionGrasp::fillBottleFromVector(const yarp::sig::Vector &vect, yarp::os::Bottle *b)
{
    for (unsigned int i=0; i<vect.size(); i++)
        b->addDouble(vect[i]);
}

/************************************************************************/
void PrecisionGrasp::writeBestSolution()
{
    std::vector<yarp::sig::Vector> contacts_r;
    std::vector<yarp::sig::Vector> normals_r;

    if (winner_triplet==1)
    {
        contacts_r=contacts_r1;
        normals_r=normals_r1;
    }
    else if (winner_triplet==2)
    {
        contacts_r=contacts_r2;
        normals_r=normals_r2;
    }
    else if (winner_triplet==3)
    {
        contacts_r=contacts_r3;
        normals_r=normals_r3;
    }
    else
    {
        contacts_r=contacts_r4;
        normals_r=normals_r4;
    }

    Bottle &out=toMatlab.prepare();
    out.clear();
    out.addString("best");
    Bottle &res=out.addList();
    Bottle &joints=res.addList();
    joints.addString("joints");
    Bottle &jointsVal=joints.addList();
    fillBottleFromVector(winner_joints, &jointsVal);

    Bottle &ee=res.addList();
    ee.addString("ee");
    Bottle &eeVal=ee.addList();
    fillBottleFromVector(winner_ee, &eeVal);

    Bottle &xdes=res.addList();
    xdes.addString("xdhat");
    Bottle &xdesVal=xdes.addList();
    fillBottleFromVector(winner_xdhat, &xdesVal);

    Bottle &axisangle=res.addList();
    axisangle.addString("axisangle");
    Bottle &axisangleVal=axisangle.addList();
    fillBottleFromVector(winner_axis, &axisangleVal);

    Bottle &odes=res.addList();
    odes.addString("odhat");
    Bottle &odesVal=odes.addList();
    fillBottleFromVector(winner_odhat, &odesVal);

    Bottle &c=res.addList();
    c.addString("center");
    Bottle &centerValue=c.addList();
    fillBottleFromVector(center, &centerValue);

    Bottle &d=res.addList();
    d.addString("dim");
    Bottle &dimValue=d.addList();
    fillBottleFromVector(dim, &dimValue);

    Bottle &c1=res.addList();
    c1.addString("c1");
    Bottle &c1Value=c1.addList();
    c1Value.addDouble(contacts_r.at(winner_combination[0])[0]);
    c1Value.addDouble(contacts_r.at(winner_combination[0])[1]);
    c1Value.addDouble(contacts_r.at(winner_combination[0])[2]);

    Bottle &c2=res.addList();
    c2.addString("c2");
    Bottle &c2Value=c2.addList();
    c2Value.addDouble(contacts_r.at(winner_combination[1])[0]);
    c2Value.addDouble(contacts_r.at(winner_combination[1])[1]);
    c2Value.addDouble(contacts_r.at(winner_combination[1])[2]);

    Bottle &c3=res.addList();
    c3.addString("c3");
    Bottle &c3Value=c3.addList();
    c3Value.addDouble(contacts_r.at(winner_combination[2])[0]);
    c3Value.addDouble(contacts_r.at(winner_combination[2])[1]);
    c3Value.addDouble(contacts_r.at(winner_combination[2])[2]);

    Bottle &n1=res.addList();
    n1.addString("n1");
    Bottle &n1Value=n1.addList();
    n1Value.addDouble(normals_r.at(winner_combination[0])[0]);
    n1Value.addDouble(normals_r.at(winner_combination[0])[1]);
    n1Value.addDouble(normals_r.at(winner_combination[0])[2]);

    Bottle &n2=res.addList();
    n2.addString("n2");
    Bottle &n2Value=n2.addList();
    n2Value.addDouble(normals_r.at(winner_combination[1])[0]);
    n2Value.addDouble(normals_r.at(winner_combination[1])[1]);
    n2Value.addDouble(normals_r.at(winner_combination[1])[2]);

    Bottle &n3=res.addList();
    n3.addString("n3");
    Bottle &n3Value=n3.addList();
    n3Value.addDouble(normals_r.at(winner_combination[2])[0]);
    n3Value.addDouble(normals_r.at(winner_combination[2])[1]);
    n3Value.addDouble(normals_r.at(winner_combination[2])[2]);

    Bottle &r=res.addList();
    r.addString("rotmat");
    Bottle &rotValue=r.addList();
    for (int i=0; i<rotation.rows(); i++)
    {
        for (int j=0; j<rotation.cols(); j++)
        {
            rotValue.addDouble(rotation(i,j));
        }
    }

    Bottle &h=res.addList();
    h.addString("hand");
    Bottle &hValue=h.addList();
    if (winner_hand=="right")
        hValue.addString("r");
    else
        hValue.addString("l");

    Bottle &cl=res.addList();
    cl.addString("cloud");
    Bottle &clValue=cl.addList();
    for (int i=0; i<cloud->size(); i++)
    {
        Bottle &point=clValue.addList();
        point.addDouble(cloud->at(i).x);
        point.addDouble(cloud->at(i).y);
        point.addDouble(cloud->at(i).z);
        point.addDouble(cloud->at(i).r);
        point.addDouble(cloud->at(i).g);
        point.addDouble(cloud->at(i).b);
    }
    if (toMatlab.getOutputCount()>0)
        toMatlab.writeStrict();
}

/************************************************************************/
void PrecisionGrasp::writeToMatlab(const std::vector<yarp::sig::Vector> &contacts_r, const std::vector<yarp::sig::Vector> &normals_r, const std::string &hand)
{
    Bottle &out=toMatlab.prepare();
    out.clear();
    Bottle &res=out.addList();
    Bottle &joints=res.addList();
    joints.addString("joints");
    Bottle &jointsVal=joints.addList();
    fillBottleFromVector(joints_tmp, &jointsVal);

    Bottle &ee=res.addList();
    ee.addString("ee");
    Bottle &eeVal=ee.addList();
    fillBottleFromVector(ee_tmp, &eeVal);

    Bottle &axisangle=res.addList();
    axisangle.addString("axisangle");
    Bottle &axisangleVal=axisangle.addList();
    fillBottleFromVector(axis_angle_tmp, &axisangleVal);

    Bottle &c=res.addList();
    c.addString("center");
    Bottle &centerValue=c.addList();
    fillBottleFromVector(center, &centerValue);

    Bottle &d=res.addList();
    d.addString("dim");
    Bottle &dimValue=d.addList();
    fillBottleFromVector(dim, &dimValue);

    Bottle &c1=res.addList();
    c1.addString("c1");
    Bottle &c1Value=c1.addList();
    c1Value.addDouble(contacts_r.at(combination_tmp[0])[0]);
    c1Value.addDouble(contacts_r.at(combination_tmp[0])[1]);
    c1Value.addDouble(contacts_r.at(combination_tmp[0])[2]);

    Bottle &c2=res.addList();
    c2.addString("c2");
    Bottle &c2Value=c2.addList();
    c2Value.addDouble(contacts_r.at(combination_tmp[1])[0]);
    c2Value.addDouble(contacts_r.at(combination_tmp[1])[1]);
    c2Value.addDouble(contacts_r.at(combination_tmp[1])[2]);

    Bottle &c3=res.addList();
    c3.addString("c3");
    Bottle &c3Value=c3.addList();
    c3Value.addDouble(contacts_r.at(combination_tmp[2])[0]);
    c3Value.addDouble(contacts_r.at(combination_tmp[2])[1]);
    c3Value.addDouble(contacts_r.at(combination_tmp[2])[2]);

    Bottle &n1=res.addList();
    n1.addString("n1");
    Bottle &n1Value=n1.addList();
    n1Value.addDouble(normals_r.at(combination_tmp[0])[0]);
    n1Value.addDouble(normals_r.at(combination_tmp[0])[1]);
    n1Value.addDouble(normals_r.at(combination_tmp[0])[2]);

    Bottle &n2=res.addList();
    n2.addString("n2");
    Bottle &n2Value=n2.addList();
    n2Value.addDouble(normals_r.at(combination_tmp[1])[0]);
    n2Value.addDouble(normals_r.at(combination_tmp[1])[1]);
    n2Value.addDouble(normals_r.at(combination_tmp[1])[2]);

    Bottle &n3=res.addList();
    n3.addString("n3");
    Bottle &n3Value=n3.addList();
    n3Value.addDouble(normals_r.at(combination_tmp[2])[0]);
    n3Value.addDouble(normals_r.at(combination_tmp[2])[1]);
    n3Value.addDouble(normals_r.at(combination_tmp[2])[2]);

    Bottle &r=res.addList();
    r.addString("rotmat");
    Bottle &rotValue=r.addList();
    for (int i=0; i<rotation.rows(); i++)
    {
        for (int j=0; j<rotation.cols(); j++)
        {
            rotValue.addDouble(rotation(i,j));
        }
    }

    Bottle &h=res.addList();
    h.addString("hand");
    Bottle &hValue=h.addList();
    if (hand=="right")
        hValue.addString("r");
    else
        hValue.addString("l");

    Bottle &cl=res.addList();
    cl.addString("cloud");
    Bottle &clValue=cl.addList();
    for (int i=0; i<cloud->size(); i++)
    {
        Bottle &point=clValue.addList();
        point.addDouble(cloud->at(i).x);
        point.addDouble(cloud->at(i).y);
        point.addDouble(cloud->at(i).z);
        point.addDouble(cloud->at(i).r);
        point.addDouble(cloud->at(i).g);
        point.addDouble(cloud->at(i).b);
    }
    if (toMatlab.getOutputCount()>0)
        toMatlab.writeStrict();
}

/************************************************************************/
void PrecisionGrasp::writeIKBestresult()
{
    std::vector<yarp::sig::Vector> contacts_r;
    std::vector<yarp::sig::Vector> normals_r;

    if (winner_triplet==1)
    {
        contacts_r=contacts_r1;
        normals_r=normals_r1;
    }
    else if (winner_triplet==2)
    {
        contacts_r=contacts_r2;
        normals_r=normals_r2;
    }
    else if (winner_triplet==3)
    {
        contacts_r=contacts_r3;
        normals_r=normals_r3;
    }
    else
    {
        contacts_r=contacts_r4;
        normals_r=normals_r4;
    }

    std::string filename="C:/Users/Utente/Desktop/IKResults/BestIKresult.txt";
    ofstream file;
    file.open(filename.c_str());
    file << "joints=[";
    for (unsigned int i=0; i<winner_joints.size(); i++)
        file << winner_joints[i] << " ";
    file <<"];\n";
    file << "ee=[" << winner_ee[0] << " " << winner_ee[1] << " " << winner_ee[2] << "];\n";
    file << "xdhat=[" << winner_xdhat[0] << " " << winner_xdhat[1] << " " << winner_xdhat[2] << "];\n";
    file << "axisangle=[" << winner_axis[0] << " " << winner_axis[1] << " " << winner_axis[2] << " " << winner_axis[3] << "];\n";
    file << "odhat=[" << winner_odhat[0] << " " << winner_odhat[1] << " " << winner_odhat[2] << " " << winner_odhat[3] << "];\n";
    file << "center=[" << center[0] << " " << center[1] << " " << center[2] << "];\n";
    file << "dim=[" << dim[0] << " " << dim[1] << " " << dim[2] << "];\n";
    file << "c1=[" << contacts_r.at(winner_combination[0])[0] << " " << contacts_r.at(winner_combination[0])[1] << " " << contacts_r.at(winner_combination[0])[2] << "];\n";
    file << "c2=[" << contacts_r.at(winner_combination[1])[0] << " " << contacts_r.at(winner_combination[1])[1] << " " << contacts_r.at(winner_combination[1])[2] << "];\n";
    file << "c3=[" << contacts_r.at(winner_combination[2])[0] << " " << contacts_r.at(winner_combination[2])[1] << " " << contacts_r.at(winner_combination[2])[2] << "];\n";
    file << "n1=[" << normals_r.at(winner_combination[0])[0] << " " << normals_r.at(winner_combination[0])[1] << " " << normals_r.at(winner_combination[0])[2] << "];\n";
    file << "n2=[" << normals_r.at(winner_combination[1])[0] << " " << normals_r.at(winner_combination[1])[1] << " " << normals_r.at(winner_combination[1])[2] << "];\n";
    file << "n3=[" << normals_r.at(winner_combination[2])[0] << " " << normals_r.at(winner_combination[2])[1] << " " << normals_r.at(winner_combination[2])[2] << "];\n";
    file << "rotmat=[";
    for (int i=0; i<rotation.rows(); i++)
    {
        for (int j=0; j<rotation.cols(); j++)
        {
            file << rotation(i,j);
            if (j!=rotation.cols()-1)
                file << " ";
        }
        if (i!=rotation.rows()-1)
            file << ";\n";
    }

    file << "];\n";
    if (winner_hand=="right")
        file << "hand='r';\n";
    else
        file << "hand='l';\n";

    file << "ov_cones " << winner_ov_cones << ";\n";

    file << "center=(" << center[0] << " " << center[1] << " " << center[2] << "];\n";
    file << "dim=(" << dim[0] << " " << dim[1] << " " << dim[2] << "];\n";
    file << "c1=(" << contacts_r.at(winner_combination[0])[0] << " " << contacts_r.at(winner_combination[0])[1] << " " << contacts_r.at(winner_combination[0])[2] << ")\n";
    file << "c2=(" << contacts_r.at(winner_combination[1])[0] << " " << contacts_r.at(winner_combination[1])[1] << " " << contacts_r.at(winner_combination[1])[2] << ")\n";
    file << "c3=(" << contacts_r.at(winner_combination[2])[0] << " " << contacts_r.at(winner_combination[2])[1] << " " << contacts_r.at(winner_combination[2])[2] << ")\n";
    file << "n1=(" << normals_r.at(winner_combination[0])[0] << " " << normals_r.at(winner_combination[0])[1] << " " << normals_r.at(winner_combination[0])[2] << ")\n";
    file << "n2=(" << normals_r.at(winner_combination[1])[0] << " " << normals_r.at(winner_combination[1])[1] << " " << normals_r.at(winner_combination[1])[2] << ")\n";
    file << "n3=(" << normals_r.at(winner_combination[2])[0] << " " << normals_r.at(winner_combination[2])[1] << " " << normals_r.at(winner_combination[2])[2] << ")\n";

    if (file.is_open())
        file.close();
}

/************************************************************************/
void PrecisionGrasp::writeIKresult(const std::vector<yarp::sig::Vector> &contacts_r, const std::vector<yarp::sig::Vector> &normals_r, const int c, const string &hand, const int ov_cones)
{
    stringstream ss;
    ss << c;
    std::string filename="C:/Users/Utente/Desktop/IKResults/IKresult_"+hand+ss.str()+".txt";
    ofstream file;
    file.open(filename.c_str());
    file << "joints=[";
    for (unsigned int i=0; i<joints_tmp.size(); i++)
        file << joints_tmp[i] << " ";
    file <<"];\n";
    file << "ee=[" << ee_tmp[0] << " " << ee_tmp[1] << " " << ee_tmp[2] << "];\n";
    file << "axisangle=[" << axis_angle_tmp[0] << " " << axis_angle_tmp[1] << " " << axis_angle_tmp[2] << " " << axis_angle_tmp[3] << "];\n";
    file << "center=[" << center[0] << " " << center[1] << " " << center[2] << "];\n";
    file << "dim=[" << dim[0] << " " << dim[1] << " " << dim[2] << "];\n";
    file << "c1=[" << contacts_r.at(combination_tmp[0])[0] << " " << contacts_r.at(combination_tmp[0])[1] << " " << contacts_r.at(combination_tmp[0])[2] << "];\n";
    file << "c2=[" << contacts_r.at(combination_tmp[1])[0] << " " << contacts_r.at(combination_tmp[1])[1] << " " << contacts_r.at(combination_tmp[1])[2] << "];\n";
    file << "c3=[" << contacts_r.at(combination_tmp[2])[0] << " " << contacts_r.at(combination_tmp[2])[1] << " " << contacts_r.at(combination_tmp[2])[2] << "];\n";
    file << "n1=[" << normals_r.at(combination_tmp[0])[0] << " " << normals_r.at(combination_tmp[0])[1] << " " << normals_r.at(combination_tmp[0])[2] << "];\n";
    file << "n2=[" << normals_r.at(combination_tmp[1])[0] << " " << normals_r.at(combination_tmp[1])[1] << " " << normals_r.at(combination_tmp[1])[2] << "];\n";
    file << "n3=[" << normals_r.at(combination_tmp[2])[0] << " " << normals_r.at(combination_tmp[2])[1] << " " << normals_r.at(combination_tmp[2])[2] << "];\n";
    file << "rotmat=[";
    for (int i=0; i<rotation.rows(); i++)
    {
        for (int j=0; j<rotation.cols(); j++)
        {
            file << rotation(i,j);
            if (j!=rotation.cols()-1)
                file << " ";
        }
        if (i!=rotation.rows()-1)
            file << ";\n";
    }

    file << "];\n";
    if (hand=="right")
        file << "hand='r';\n";
    else
        file << "hand='l';\n";

    file << "ov_cones " << ov_cones << ";\n";

    file << "center=(" << center[0] << " " << center[1] << " " << center[2] << ")\n";
    file << "dim=(" << dim[0] << " " << dim[1] << " " << dim[2] << ")\n";
    file << "c1=(" << contacts_r.at(combination_tmp[0])[0] << " " << contacts_r.at(combination_tmp[0])[1] << " " << contacts_r.at(combination_tmp[0])[2] << ")\n";
    file << "c2=(" << contacts_r.at(combination_tmp[1])[0] << " " << contacts_r.at(combination_tmp[1])[1] << " " << contacts_r.at(combination_tmp[1])[2] << ")\n";
    file << "c3=(" << contacts_r.at(combination_tmp[2])[0] << " " << contacts_r.at(combination_tmp[2])[1] << " " << contacts_r.at(combination_tmp[2])[2] << ")\n";
    file << "n1=(" << normals_r.at(combination_tmp[0])[0] << " " << normals_r.at(combination_tmp[0])[1] << " " << normals_r.at(combination_tmp[0])[2] << ")\n";
    file << "n2=(" << normals_r.at(combination_tmp[1])[0] << " " << normals_r.at(combination_tmp[1])[1] << " " << normals_r.at(combination_tmp[1])[2] << ")\n";
    file << "n3=(" << normals_r.at(combination_tmp[2])[0] << " " << normals_r.at(combination_tmp[2])[1] << " " << normals_r.at(combination_tmp[2])[2] << ")\n";

    if (file.is_open())
        file.close();
}

/************************************************************************/
bool PrecisionGrasp::respond(const Bottle& command, Bottle& reply) 
{
    string tag_0=command.get(0).asString().c_str();
    if (tag_0=="set")
    {
        if (command.size()<3)
        {
            reply.addString("nack, command not recognized");
            return true;
        }
        string tag_1=command.get(1).asString().c_str();
        if (tag_1=="visualization")
        {
            if (command.get(2).asString()=="on")
                visualize=true;
            else
                visualize=false;
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="filter")
        {
            if (command.get(2).asString()=="on")
                filterCloud=true;
            else
                filterCloud=false;
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="x")
        {
            if (command.size()>1)
            {
                posx=command.get(1).asInt();
                visualizationThread->setPosition(posx,posy);
                reply.addString("ack");
            }
            else
                reply.addString("nack");
            return true;
        }
        else if (tag_1=="y")
        {
            if (command.size()>1)
            {
                posy=command.get(1).asInt();
                visualizationThread->setPosition(posx,posy);
                reply.addString("ack");
            }
            else
                reply.addString("nack");
            return true;
        }
        else if (tag_1=="w")
        {
            if (command.size()>1)
            {
                sizex=command.get(1).asInt();
                visualizationThread->setSize(sizex,sizey);
                reply.addString("ack");
            }
            else
                reply.addString("nack");
            return true;
        }
        else if (tag_1=="h")
        {
            if (command.size()>1)
            {
                sizey=command.get(1).asInt();
                visualizationThread->setSize(sizex,sizey);
                reply.addString("ack");
            }
            else
                reply.addString("nack");
            return true;
        }
        else if (tag_1=="write")
        {
            if (command.get(2).asString()=="on")
                writeCloud=true;
            else
                writeCloud=false;
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="offsetR")
        {
            offsetR[0]=command.get(2).asDouble();
            offsetR[1]=command.get(3).asDouble();
            offsetR[2]=command.get(4).asDouble();
            reply.addString("ack");
            return true;
        }
        else if (tag_1=="offsetL")
        {
            offsetL[0]=command.get(2).asDouble();
            offsetL[1]=command.get(3).asDouble();
            offsetL[2]=command.get(4).asDouble();
            reply.addString("ack");
            return true;
        }
    }
    if (tag_0=="IK1" || tag_0=="IK2" || tag_0=="IK3" || tag_0=="IK4")
    {
        if (current_state==STATE_IK)
        {
            counter--;
            reply.addString("ack");
            if (tag_0=="IK1")
            {
                mutex_to_write.wait();
                string hand=extractData(command,1);                
                //writeToMatlab(contacts_r1,normals_r1,hand);
                writeIKresult(contacts_r1,normals_r1,1,hand,ov_cones1);
                mutex_to_write.post();
            }
            else if (tag_0=="IK2")
            {
                mutex_to_write.wait();
                string hand=extractData(command,2);
                //writeToMatlab(contacts_r2,normals_r2,hand);
                writeIKresult(contacts_r2,normals_r2,2,hand,ov_cones2);
                mutex_to_write.post();
            }
            else if (tag_0=="IK3")
            {
                mutex_to_write.wait();
                string hand=extractData(command,3);
                //writeToMatlab(contacts_r3,normals_r3,hand);
                writeIKresult(contacts_r3,normals_r3,3,hand,ov_cones3);
                mutex_to_write.post();
            }
            else
            {
                mutex_to_write.wait();
                string hand=extractData(command,4);
                //writeToMatlab(contacts_r4,normals_r4,hand);
                writeIKresult(contacts_r4,normals_r4,4,hand,ov_cones4);
                mutex_to_write.post();
            }
            if (counter==0)
            {
                //Time::delay(1.0);

                writeBestSolution();
                writeIKBestresult();

                //Time::delay(1.0);

                bestCost=1e20;
                bestManipulability=0.0;

                iCtrlRight->restoreContext(current_context_right);
                iCtrlLeft->restoreContext(current_context_left);

                readyToGrasp=true;
                if (!dont && straight)
                {
                    current_state=STATE_GRASP;
                    straight=false;
                }
                else    
                    current_state=STATE_WAIT;
                printf("ready\n");
            }
        }
        else
            reply.addString("nack");
        return true;
    }
    if (tag_0=="go")
    {
        if (readyToGrasp)
        {
            current_state=STATE_GRASP;
            eventRpc.reset();
            eventRpc.wait();
            reply.addString("ack");
        }
        else
            reply.addString("nack");
        return true;
    }
    if (tag_0=="help")
    {
        reply.addString("set visualization on/off");
        reply.addString("set x");
        reply.addString("set y");
        reply.addString("set w");
        reply.addString("set h");
        reply.addString("set offsetL x y z");
        reply.addString("set offsetR x y z");
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
    if (tag_0=="block")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=true;
            else
                leftBlocked=true;
            reply.addString("ack");
            return true;
        }
        else
        {
            reply.addString("nack");
            return true;
        }
    }
    if (tag_0=="unblock")
    {
        if (command.size()>=2)
        {
            if (command.get(1).asString()=="right")
                rightBlocked=false;
            else
                leftBlocked=false;
            reply.addString("ack");
            return true;
        }
        else
        {
            reply.addString("nack");
            return true;
        }
    }
    if (tag_0=="isGrasped")
    {
        string r=grasped?"true":"false";
        reply.addString(r.c_str());
        return true;
    }
    if (tag_0=="dont")
    {
        dont=true;
        readyToGrasp=false;
        reply.addString("ack");
        current_state=STATE_WAIT;
        return true;
    }
    if (tag_0=="grasp")
    {
        if (current_state==STATE_ESTIMATE || current_state==STATE_IK || current_state==STATE_GRASP)
        {
            reply.addString("nack");
            return true;
        }

        if (visualizationThread->isRunning())
            visualizationThread->stop();
                
        readyToGrasp=false;
        grasped=false;
        dont=false;

        Bottle *pos=command.get(1).asList();

        Bottle cmd1;
        reply.clear();
        cmd1.addInt(pos->get(0).asInt());
        cmd1.addInt(pos->get(1).asInt());

        if (reconstructionPort.getOutputCount()>0)
            reconstructionPort.write(cmd1,reply);

        Bottle cmd2;
        reply.clear();
        cmd2.addString("3Drec");

        if (reconstructionPort.getOutputCount()>0)
            reconstructionPort.write(cmd2,reply);
                
        if (reply.size()>0 && reply.get(0).asString()=="ack")
        {
            reply.clear();
            current_state=STATE_ESTIMATE;
            straight=true;
            if (command.size()>=2)
                straight=(command.get(2).asString()!="wait");
            if (straight)
            {
                eventRpc.reset();
                eventRpc.wait();
            }
            if (grasped)
                reply.addString("ack");
            else
                reply.addString("nack");
        }
        else
            reply.addString("nack");
                
        return true;
    }
    reply.addString("nack");
    return true;
}

/************************************************************************/
void PrecisionGrasp::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
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
double PrecisionGrasp::getPeriod()
{
    return 0.1;
}

/************************************************************************/
bool PrecisionGrasp::normalPointingOut(pcl::Normal &normal, pcl::PointXYZ &point)
{
    yarp::sig::Vector p(3);
    p[0]=point.x-(normal.normal_x*0.003);
    p[1]=point.y-(normal.normal_y*0.003);
    p[2]=point.z-(normal.normal_z*0.003);

    yarp::sig::Vector fromCenter=(center-p)/norm(center-p);

    yarp::sig::Vector n(3);
    n[0]=normal.normal_x;
    n[1]=normal.normal_y;
    n[2]=normal.normal_z;
    n=n/norm(n);
    
    return (dot(n,fromCenter)<0);
}

