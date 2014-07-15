#include "handIKModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

HandIKModule::HandIKModule()
{
    done=true;
    work=false;
}

bool HandIKModule::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("handIKModule1")).asString().c_str();
    hand=rf.check("hand",Value("right")).asString().c_str();
    /*hand=rf.find("hand").asString().c_str();
    if (hand!="right" && hand!="left")
        return false;*/
    portName="/"+name+"/"+hand+"/out";
    outputPort.open(portName.c_str());
    string rpcPortName="/"+name+"/"+hand+"/rpc";
    rpc.open(rpcPortName.c_str());
    attach(rpc);
    createCombinationVector();
    
    return true;
}

bool HandIKModule::extractData(const Bottle &data)
{
    Bottle &b=const_cast<Bottle&>(data);
    Bottle* c=b.find("center").asList();
    fillVectorFromBottle(c,center);
    Bottle* d=b.find("dim").asList();
    fillVectorFromBottle(d,dim);
    Bottle* c1B=b.find("c1").asList();
    yarp::sig::Vector c1;
    fillVectorFromBottle(c1B,c1);
    c1.push_back(1.0);
    Bottle* c2B=b.find("c2").asList();
    yarp::sig::Vector c2;
    fillVectorFromBottle(c2B,c2);
    c2.push_back(1.0);
    Bottle* c3B=b.find("c3").asList();
    yarp::sig::Vector c3;
    fillVectorFromBottle(c3B,c3);
    c3.push_back(1.0);
    Bottle* n1B=b.find("n1").asList();
    yarp::sig::Vector n1;
    fillVectorFromBottle(n1B,n1);
    Bottle* n2B=b.find("n2").asList();
    yarp::sig::Vector n2;
    fillVectorFromBottle(n2B,n2);
    Bottle* n3B=b.find("n3").asList();
    yarp::sig::Vector n3;
    fillVectorFromBottle(n3B,n3);
    Bottle* rot=b.find("rot").asList();
    fillMatrixFromBottle(rot,rotation,3,3);
    contacts_r.push_back(c1);
    contacts_r.push_back(c2);
    contacts_r.push_back(c3);
    normals_r.push_back(n1);
    normals_r.push_back(n2);
    normals_r.push_back(n3);
    return true;
}

void HandIKModule::fillMatrixFromBottle(const Bottle* b, yarp::sig::Matrix &m, int rows, int cols)
{
    int k=0;
    m.resize(rows,cols);
    for (int i=0; i<rows; i++)
    {
        for (int j=0; j<cols; j++)
        {
            m(i,j)=b->get(k).asDouble();
            k++;
        }
    }
}

void HandIKModule::fillVectorFromBottle(const Bottle* b, yarp::sig::Vector &v)
{
    v.resize(b->size());
    for (int i=0; i<b->size(); i++)
        v[i]=b->get(i).asDouble();
}

bool HandIKModule::interruptModule()
{
	printf("interrupting\n");
    outputPort.interrupt();
    printf("interrupted out\n");
    rpc.interrupt();
    printf("interrupted rpc\n");
    return true;
}

bool HandIKModule::close()
{
	printf("closing\n");
    outputPort.close();
    printf("closed out\n");
    rpc.close();
    printf("closed rpc\n");
    return true;
}

bool HandIKModule::updateModule()
{
    if (work)
    {
        bestObjValue=10000;
        int index=0;

        for (unsigned int i=0; i<combinations.size(); i++)
        {
            HandIK_Problem problem(hand,3);

            problem.dimensions[0]=dim[1];
            problem.dimensions[1]=dim[0];
            problem.dimensions[2]=dim[2];

            problem.contactPoints.push_back(contacts_o.at((unsigned int)combinations.at(i)[0]));
            problem.contactPoints.push_back(contacts_o.at((unsigned int)combinations.at(i)[1]));
            problem.contactPoints.push_back(contacts_o.at((unsigned int)combinations.at(i)[2]));

            problem.normalDirs.push_back(normals_o.at((unsigned int)combinations.at(i)[0]));
            problem.normalDirs.push_back(normals_o.at((unsigned int)combinations.at(i)[1]));
            problem.normalDirs.push_back(normals_o.at((unsigned int)combinations.at(i)[2]));

            HandIK_Solver solver(problem);

            //TODO: setting initial orientation in a smarter way
            HandIK_Variables guess(problem.nFingers);
            guess.xyz_ee[0]=0.0;
            guess.xyz_ee[1]=0.0;
            guess.xyz_ee[2]=0.1;

            if (hand=="right")
            {
                // the following orientation corresponds to the right hand
                // put palm down on top of the object, with the middle finger
                // aligned wrt the object y-axis
                guess.rpy_ee[0]=-M_PI;
                guess.rpy_ee[1]=0.0;
                guess.rpy_ee[2]=M_PI/2.0;
            }
            else
            {
                guess.rpy_ee[0]=0.0;
                guess.rpy_ee[1]=0.0;
                guess.rpy_ee[2]=M_PI;
            }
            guess.joints=0.0;
            solver.setInitialGuess(guess);
            //printf("#### initial guess ...\n");
            //guess.print();

            HandIK_Variables solution;
            double t0=Time::now();
            bool found=solver.solve(solution);
            double t1=Time::now();

            printf("\n");
            printf("#### %d solution found in %g [s] ...\n", i, t1-t0);

            double objFunc=solution.cost_fun;
            printf("Cost %g\n", objFunc);
            objFunc+=evaluateFingers(solution,i);
            printf("Global cost %g\n", objFunc);
            if (objFunc<bestObjValue)
            {
                bestObjValue=objFunc;
                index=i;
                bestSolution=solution;
            }
        }
        winnerIndex=index;
        bestSolution.print();

        Bottle out;
        prepareData(out);

        outputPort.write(out);

        work=false;
        done=true;
    }
    return true;
}

double HandIKModule::evaluateFingers(const HandIK_Variables &solution, const int id)
{
    yarp::sig::Vector encoders(9,0.0);
    encoders[0]=solution.joints[3];
    encoders[1]=solution.joints[0];
    encoders[2]=solution.joints[1];
    encoders[3]=solution.joints[2];
    encoders[4]=solution.joints[4];
    encoders[5]=solution.joints[5];
    encoders[6]=solution.joints[6];
    encoders[7]=solution.joints[7];

    iCub::iKin::iCubFinger thumb(hand+"_thumb");
    iCub::iKin::iCubFinger index(hand+"_index");
    iCub::iKin::iCubFinger middle(hand+"_middle");

    yarp::sig::Vector joints_thumb;
    thumb.getChainJoints(encoders,joints_thumb);
    yarp::sig::Matrix thumbH=thumb.getH(joints_thumb);
    yarp::sig::Vector thumbH_col3=thumbH.getCol(3);

    yarp::sig::Vector joints_index;
    index.getChainJoints(encoders,joints_index);
    yarp::sig::Matrix indexH=index.getH(joints_index);
    yarp::sig::Vector indexH_col3=indexH.getCol(3);
    
    yarp::sig::Vector joints_middle;
    middle.getChainJoints(encoders,joints_middle);
    yarp::sig::Matrix middleH=middle.getH(joints_middle);
    yarp::sig::Vector middleH_col3=middleH.getCol(3);

    yarp::sig::Matrix Hc=rpy2dcm(solution.rpy_ee);
    Hc(0,3)=solution.xyz_ee[0];
    Hc(1,3)=solution.xyz_ee[1];
    Hc(2,3)=solution.xyz_ee[2];

    double res=0.0;
    res+=norm2(contacts_o.at(combinations.at(id)[0])-(Hc*thumbH_col3));
    res+=norm2(contacts_o.at(combinations.at(id)[1])-(Hc*indexH_col3));
    res+=norm2(contacts_o.at(combinations.at(id)[2])-(Hc*middleH_col3));
    res*=1e04;
    printf("res %g\n", res);

    return res;
}

void HandIKModule::fromRootToObject()
{
    /*Matrix rot_tran=zeros(4,4);

    rot_tran.setSubmatrix(rotation,0,0);
    rot_tran(3,3)=1.0;
    rot_tran(0,3)=center[0];
    rot_tran(1,3)=center[1];
    rot_tran(2,3)=center[2];
    Matrix rotmat=SE3inv(rot_tran);*/

    rot_tran=zeros(4,4);
    
    rot_tran(1,0)=1.0;
    rot_tran(0,1)=-1.0;
    rot_tran(2,2)=1.0;
    rot_tran(3,3)=1.0;
    rot_tran(0,3)=center[0];
    rot_tran(1,3)=center[1];
    rot_tran(2,3)=center[2];
    Matrix rotmat=SE3inv(rot_tran);

    Vector c1=rotmat*contacts_r.at(0);
    Vector c2=rotmat*contacts_r.at(1);
    Vector c3=rotmat*contacts_r.at(2);
    Vector n1=rotmat.submatrix(0,2,0,2)*normals_r.at(0).subVector(0,2);
    n1.push_back(1.0);
    Vector n2=rotmat.submatrix(0,2,0,2)*normals_r.at(1).subVector(0,2);
    n2.push_back(1.0);
    Vector n3=rotmat.submatrix(0,2,0,2)*normals_r.at(2).subVector(0,2);
    n3.push_back(1.0);
    contacts_o.push_back(c1);
    contacts_o.push_back(c2);
    contacts_o.push_back(c3);
    normals_o.push_back(n1);
    normals_o.push_back(n2);
    normals_o.push_back(n3);
}

void HandIKModule::createCombinationVector()
{
    Vector v1(3); v1[0]=0; v1[1]=1; v1[2]=2;
    Vector v2(3); v2[0]=0; v2[1]=2; v2[2]=1;
    Vector v3(3); v3[0]=1; v3[1]=0; v3[2]=2;
    Vector v4(3); v4[0]=1; v4[1]=2; v4[2]=0;
    Vector v5(3); v5[0]=2; v5[1]=1; v5[2]=0;
    Vector v6(3); v6[0]=2; v6[1]=0; v6[2]=1;
    combinations.push_back(v1);
    combinations.push_back(v2);
    combinations.push_back(v3);
    combinations.push_back(v4);
    combinations.push_back(v5);
    combinations.push_back(v6);
}

bool HandIKModule::respond(const Bottle& command, Bottle& reply) 
{
    tag=command.get(0).asString();
    if (tag=="IK1" || tag=="IK2" || tag=="IK3" || tag=="IK4")
    {
        Network::connect(portName.c_str(),"/precisionGrasp/rpc");
        contacts_r.clear();
        normals_r.clear();
        contacts_o.clear();
        normals_o.clear();
        bool f=extractData(command);
        if (f)
        {
            fromRootToObject();
            work=true;
            reply.addString("ack");
            return true;
        }
    }
    reply.addString("nack");
    return true;
}

void HandIKModule::prepareData(yarp::os::Bottle &data)
{
    data.addString(tag.c_str());
    Bottle &handB=data.addList();
    handB.addString("hand");
    handB.addString(hand.c_str());
    Bottle &cost=data.addList();
    cost.addString("cost");
    cost.addDouble(bestObjValue);

    rot_tran=zeros(4,4);
    
    rot_tran(1,0)=1.0;
    rot_tran(0,1)=-1.0;
    rot_tran(2,2)=1.0;
    rot_tran(3,3)=1.0;
    rot_tran(0,3)=center[0];
    rot_tran(1,3)=center[1];
    rot_tran(2,3)=center[2];

    yarp::sig::Vector ee_ob=bestSolution.xyz_ee;
    ee_ob.push_back(1.0);

    yarp::sig::Vector ee_root=rot_tran*ee_ob;
    yarp::sig::Matrix tmp=rpy2dcm(bestSolution.rpy_ee);
    yarp::sig::Matrix tmp2=rot_tran*tmp;
    yarp::sig::Vector or_root=dcm2axis(tmp2);

    Bottle &ee=data.addList();
    ee.addString("ee");
    Bottle &ee_coord=ee.addList();
    ee_coord.addDouble(ee_root[0]);
    ee_coord.addDouble(ee_root[1]);
    ee_coord.addDouble(ee_root[2]);
    Bottle &orientation=data.addList();
    orientation.addString("or");
    Bottle &or_coord=orientation.addList();
    yarp::sig::Vector or_axisangle=dcm2axis(rpy2dcm(bestSolution.rpy_ee));
    or_coord.addDouble(or_root[0]);
    or_coord.addDouble(or_root[1]);
    or_coord.addDouble(or_root[2]);
    or_coord.addDouble(or_root[3]);
    Bottle &j=data.addList();
    j.addString("joints");
    Bottle &joints=j.addList();
    for (unsigned int i=0; i<bestSolution.joints.size(); i++)
        joints.addDouble(bestSolution.joints[i]);
    Bottle &order=data.addList();
    order.addString("combination");
    Bottle &combination=order.addList();
    combination.addInt((int)combinations.at(winnerIndex)[0]);
    combination.addInt((int)combinations.at(winnerIndex)[1]);
    combination.addInt((int)combinations.at(winnerIndex)[2]);
}

double HandIKModule::getPeriod()
{
    return 0.1;
}

