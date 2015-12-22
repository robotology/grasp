
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cstdlib>

#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/math.h>
#include "handIK.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

std::vector<yarp::sig::Vector> combinations;
std::vector<yarp::sig::Vector> contacts_o;
std::vector<yarp::sig::Vector> normals_o;
string hand="right";

void vectorFromBottle(Bottle* b, Vector &v)
{
    for (unsigned int i=0; i<b->size(); i++)
        v[i]=b->get(i).asDouble();
}

void readGrasp(ResourceFinder &rf, Vector &center, Vector &dim, std::vector<yarp::sig::Vector> &contacts_r, std::vector<yarp::sig::Vector> &normals_r)
{
    Bottle* centerB=rf.find("center").asList();
    vectorFromBottle(centerB,center);

    Bottle* dimB=rf.find("dim").asList();
    vectorFromBottle(dimB,dim);

    Bottle* contact1=rf.find("contact1").asList();
    Vector c1(4,1.0);
    vectorFromBottle(contact1,c1);

    Bottle* contact2=rf.find("contact2").asList();
    Vector c2(4,1.0);
    vectorFromBottle(contact2,c2);

    Bottle* contact3=rf.find("contact3").asList();
    Vector c3(4,1.0);
    vectorFromBottle(contact3,c3);

    Bottle* normal1=rf.find("normal1").asList();
    Vector n1(4,1.0);
    vectorFromBottle(normal1,n1);

    Bottle* normal2=rf.find("normal2").asList();
    Vector n2(4,1.0);
    vectorFromBottle(normal2,n2);

    Bottle* normal3=rf.find("normal3").asList();
    Vector n3(4,1.0);
    vectorFromBottle(normal3,n3);

    contacts_r.push_back(c1);
    contacts_r.push_back(c2);
    contacts_r.push_back(c3);

    normals_r.push_back(n1);
    normals_r.push_back(n2);
    normals_r.push_back(n3);
}

double evaluateFingers(const HandIK_Variables &solution, const int id)
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

int test(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("handIK");
    rf.setDefaultConfigFile("contactPoints_fitness1.ini");
    rf.configure(argc,argv);
    
    Vector center(3),dim(3);
    std::vector<yarp::sig::Vector> contacts_r;
    std::vector<yarp::sig::Vector> normals_r;

    readGrasp(rf,center,dim,contacts_r,normals_r);

    //Matrix to pass from root reference frame to the object reference frame
    
    // object reference frame:
    // x-axis along the robot  y
    // y-axis along the robot -x
    // z-axis along the robot  z
    Matrix tmp=zeros(4,4);
    
    tmp(1,0)=1.0;
    tmp(0,1)=-1.0;
    tmp(2,2)=1.0;
    tmp(3,3)=1.0;
    tmp(0,3)=center[0];
    tmp(1,3)=center[1];
    tmp(2,3)=center[2];

    Matrix rotmat=SE3inv(tmp);

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

    HandIK_Variables bestSolution;
    double bestObjValue=10000;
    int index=0;

    for (unsigned int i=0; i<combinations.size(); i++)
    {
        //HandIK_Problem problem("right",2);
        HandIK_Problem problem(hand,3);

        problem.dimensions[0]=dim[1];
        problem.dimensions[1]=dim[0];
        problem.dimensions[2]=dim[2];
        
        problem.contactPoints.push_back(contacts_o.at(combinations.at(i)[0]));
        problem.contactPoints.push_back(contacts_o.at(combinations.at(i)[1]));
        problem.contactPoints.push_back(contacts_o.at(combinations.at(i)[2]));

        problem.normalDirs.push_back(normals_o.at(combinations.at(i)[0]));
        problem.normalDirs.push_back(normals_o.at(combinations.at(i)[1]));
        problem.normalDirs.push_back(normals_o.at(combinations.at(i)[2]));

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
        /*if (i==3)
        {
            index=i;
            bestSolution=solution;
        }*/
    }

    printf("\n\nWinner index %d\n", index);
    bestSolution.print();
    printf("axis-angle_ee = (%s) [rad]\n",dcm2axis(rpy2dcm(bestSolution.rpy_ee)).toString(3,3).c_str());

    Vector joints=bestSolution.joints;
    
    Vector ee=bestSolution.xyz_ee;
    ee.push_back(1.0);
    ee=tmp*ee;
    ee=ee.subVector(0,2);
   
    Matrix r=rpy2dcm(bestSolution.rpy_ee);
    Vector axsangle=dcm2axis(tmp*r);
    
    std::string filename="C:/Users/Utente/Desktop/IKresult.txt";
    ofstream file;
    file.open(filename.c_str());

    file << "joints=[";
    for (unsigned int i=0; i<joints.size(); i++)
        file << joints[i] << " ";
    file <<"];\n";

    file << "ee=[" << ee[0] << " " << ee[1] << " " << ee[2] << "];\n";
    file << "axisangle=[" << axsangle[0] << " " << axsangle[1] << " " << axsangle[2] << " " << axsangle[3] << "];\n";
    file << "center=[" << center[0] << " " << center[1] << " " << center[2] << "];\n";
    file << "dim=[" << dim[0] << " " << dim[1] << " " << dim[2] << "];\n";
    file << "c1=[" << contacts_r.at(combinations.at(index)[0])[0] << " " << contacts_r.at(combinations.at(index)[0])[1] << " " << contacts_r.at(combinations.at(index)[0])[2] << "];\n";
    file << "c2=[" << contacts_r.at(combinations.at(index)[1])[0] << " " << contacts_r.at(combinations.at(index)[1])[1] << " " << contacts_r.at(combinations.at(index)[1])[2] << "];\n";
    file << "c3=[" << contacts_r.at(combinations.at(index)[2])[0] << " " << contacts_r.at(combinations.at(index)[2])[1] << " " << contacts_r.at(combinations.at(index)[2])[2] << "];\n";
    file << "n1=[" << normals_r.at(combinations.at(index)[0])[0] << " " << normals_r.at(combinations.at(index)[0])[1] << " " << normals_r.at(combinations.at(index)[0])[2] << "];\n";
    file << "n2=[" << normals_r.at(combinations.at(index)[1])[0] << " " << normals_r.at(combinations.at(index)[1])[1] << " " << normals_r.at(combinations.at(index)[1])[2] << "];\n";
    file << "n3=[" << normals_r.at(combinations.at(index)[2])[0] << " " << normals_r.at(combinations.at(index)[2])[1] << " " << normals_r.at(combinations.at(index)[2])[2] << "];\n";

    if (file.is_open())
        file.close();

    return 0;
}


