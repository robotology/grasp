#include <iCub/grasp/forceClosure.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

static double cross2D(const Vector &v1, const Vector &v2)
{
    return (v1[0]*v2[1])-(v1[1]*v2[0]);
}

static bool included(double cosine, double angle)
{
    double alpha1=(M_PI/2)+angle;
    double alpha2=(M_PI/2)-angle;

    double cosMin=cos(alpha1);
    double cosMax=cos(alpha2);

    return (cosine>cosMin)&&(cosine<cosMax);
}

static bool computeIntersectionPoint(const yarp::sig::Vector &line1, const yarp::sig::Vector &line2, yarp::sig::Vector &point)
{
    point.resize(2,0.0);
    if (line2[0]-line1[0]==0)
        return false;
    point[0]=(line1[1]-line2[1])/(line2[0]-line1[0]);
    point[1]=(line1[0]*point[0])+line1[1];
    return true;
}

static yarp::sig::Vector projectVectorOnThePlane(const yarp::sig::Vector &v, const yarp::sig::Vector &plane)
{
    yarp::sig::Vector projection=v-(dot(v,plane)*plane);
    projection/=norm(projection);
    return projection;
}

static void computeLineParameters(const Vector &x1, const Vector &x2, Vector &line)
{
    line.resize(2,0.0);
    line[0]=1.0e+10;

    if ((x2[0]-x1[0])!=0.0)
        line[0]=(x2[1]-x1[1])/(x2[0]-x1[0]);
        
    line[1]=x1[1]-(line[0]*x1[0]);
}

static bool pointInCone(const vector<Vector> &a1, const vector<Vector> &a2, const vector<Vector> &a3, const yarp::sig::Vector &c)
{
    Vector intersectionPoint(2);
    for (unsigned int i=0; i<a1.size(); i++)
    {
        for (unsigned int j=0; j<a2.size(); j++)
        {
            if (computeIntersectionPoint(a1[i],a2[j],intersectionPoint))
            {
                if (sign(cross2D(intersectionPoint-c,a3.at(0)))*sign(cross2D(intersectionPoint-c,a3.at(1)))<0)
                    return true;
            }
        }
    }
    return false;
}

static yarp::sig::Vector transformVector(const yarp::sig::Vector &v, const yarp::sig::Matrix &orientation)
{
    return orientation.submatrix(0,2,0,2).transposed()*v;
}

static yarp::sig::Vector transformPoint(const yarp::sig::Vector &p, const yarp::sig::Matrix &orientation)
{
    Vector pHom(4); pHom[3]=1.0;
    pHom.setSubvector(0,p);
    return (SE3inv(orientation)*pHom).subVector(0,2);
}

static bool verifySameHalfPlane(const Vector &normal1, const Vector &normal2, const Vector &normal3)
{
    Vector line1;
    Vector line2;
    Vector line3;

    Vector tmp(2); tmp=0.0;

    computeLineParameters(normal1,tmp,line1);
    computeLineParameters(normal2,tmp,line2);
    computeLineParameters(normal3,tmp,line3);

    //Sign with respect to the line on which vector normal1 lies
    double signv2=sign(normal2[1]-(line1[0]*normal2[0])-line1[1]);
    double signv3=sign(normal3[1]-(line1[0]*normal3[0])-line1[1]);

    if ((signv2==signv3)||(signv2==0)||(signv3==0))
        return false;

    //Sign with respect to the line on which vector normal2 lies
    double signv1=sign(normal1[1]-(line2[0]*normal1[0])-line2[1]);
    signv3=sign(normal3[1]-(line2[0]*normal3[0])-line2[1]);

    if ((signv1==signv3)||(signv1==0)||(signv3==0))
        return false;

    //Sign with respect to the line on which vector normal3 lies
    signv1=sign(normal1[1]-(line3[0]*normal1[0])-line3[1]);
    signv2=sign(normal2[1]-(line3[0]*normal2[0])-line3[1]);

    if ((signv1==signv2)||(signv1==0)||(signv2==0))
        return false;

    return true;
}

static bool verifyConesIntersection(const yarp::sig::Vector &c1, const yarp::sig::Vector &c2, const yarp::sig::Vector &c3, const std::vector<iCub::grasp::ConeBounds> &conesBounds2D)
{
    Vector line11, line12, line21, line22, line31, line32;

    computeLineParameters(c1+conesBounds2D.at(0).n1,c1,line11);
    computeLineParameters(c1+conesBounds2D.at(0).n2,c1,line12);

    vector<Vector> a1; a1.push_back(line11); a1.push_back(line12);
    computeLineParameters(c2+conesBounds2D.at(1).n1,c2,line21);
    computeLineParameters(c2+conesBounds2D.at(1).n2,c2,line22);

    vector<Vector> a2; a2.push_back(line21);a2.push_back(line22);
    computeLineParameters(c3+conesBounds2D.at(2).n1,c3,line31);
    computeLineParameters(c3+conesBounds2D.at(2).n2,c3,line32);

    vector<Vector> a3; a3.push_back(line31); a3.push_back(line32);
       
    if (pointInCone(a1,a2,a3,c3))
        return true;
    if (pointInCone(a1,a3,a2,c2))
        return true;
    if (pointInCone(a2,a3,a1,c1))
        return true;

    return false;
}

static Matrix findOrientationForPlane(const iCub::grasp::ContactPoints &contactPoints, const Vector &plane)
{
    Vector projection1=projectVectorOnThePlane(contactPoints.n1,plane);
    Vector c=(contactPoints.c1+contactPoints.c2+contactPoints.c3)/3;

    Vector z=plane;
    Vector x=projection1;
    Vector y=cross(plane,x); y/=norm(y);

    Matrix orientation=eye(4,4);
    orientation(0,0)=x[0]; orientation(1,0)=x[1]; orientation(2,0)=x[2];
    orientation(0,1)=y[0]; orientation(1,1)=y[1]; orientation(2,1)=y[2];
    orientation(0,2)=z[0]; orientation(1,2)=z[1]; orientation(2,2)=z[2];
    orientation(0,3)=c[0]; orientation(1,3)=c[1]; orientation(2,3)=c[2];

    return orientation;
}

static yarp::sig::Vector solveForT(const double &a, const double &b, const double &c)
{
    yarp::sig::Vector t(2);
    double normFact=sqrt((a*a)+(b*b));
    double alpha=acos(a/normFact);

    if (abs(sin(alpha)-(b/normFact))>0.0001)
        alpha=-alpha;

    double x0=asin(c/normFact);
    double x1=M_PI-asin(c/normFact);

    t[0]=x0-alpha;
    t[1]=x1-alpha;

    return t;
}

static iCub::grasp::ConeBounds findConeBounds(const yarp::sig::Vector &c, const yarp::sig::Vector &n, const yarp::sig::Vector &plane, double alpha)
{
    iCub::grasp::ConeBounds coneBounds;
    //center of the circle at cone height=1
    yarp::sig::Vector c0=c+n;
    //vector to project on the plane where the circle lies to create a basis
    yarp::sig::Vector tmp(3); tmp[0]=1.0; tmp[1]=0.0; tmp[2]=0.0;
    //basis for the plane where the circle lies
    yarp::sig::Vector x=projectVectorOnThePlane(tmp,n); x/=norm(x);
    yarp::sig::Vector y=cross(n,x); y/=norm(y);

    //radius of the circle when the height of the cone is 1
    double r=tan(alpha);

    //values to solve the system between the circle and the fact that the line between the point
    //on the circle and the contact point must belongs to the plane
    double A=r*dot(x,plane);
    double B=r*dot(y,plane);
    double C=(dot(c,plane))-(dot(c0,plane));

    //Finds the angle to which the circle meets the plane on two points
    yarp::sig::Vector t=solveForT(A,B,C);

    //Two points on the circle where it meets the plane
    yarp::sig::Vector P1=c0+(r*sin(t[0])*x)+(r*cos(t[0])*y);
    yarp::sig::Vector P2=c0+(r*sin(t[1])*x)+(r*cos(t[1])*y);

    //Cone bound vectors
    coneBounds.n1=(P1-c)/norm(P1-c);
    coneBounds.n2=(P2-c)/norm(P2-c);

    return coneBounds;
}

static bool verifyCondition2D(const iCub::grasp::ContactPoints &contactPoints, const Vector &plane, const std::vector<iCub::grasp::ConeBounds> &conesBounds)
{
    Matrix orientation=findOrientationForPlane(contactPoints,plane);

    Vector projection1=projectVectorOnThePlane(contactPoints.n1,plane);
    Vector projection2=projectVectorOnThePlane(contactPoints.n2,plane);
    Vector projection3=projectVectorOnThePlane(contactPoints.n3,plane);

    Vector n1=transformVector(projection1,orientation);
    Vector n2=transformVector(projection2,orientation);
    Vector n3=transformVector(projection3,orientation);

    Vector c1=(transformPoint(contactPoints.c1,orientation)).subVector(0,1);
    Vector c2=(transformPoint(contactPoints.c2,orientation)).subVector(0,1);
    Vector c3=(transformPoint(contactPoints.c3,orientation)).subVector(0,1);

    std::vector<iCub::grasp::ConeBounds> conesBounds2D;
    for (unsigned int i=0; i<conesBounds.size(); i++)
    {
        iCub::grasp::ConeBounds tmp;
        tmp.n1=(transformVector(conesBounds[i].n1,orientation)).subVector(0,1);
        tmp.n2=(transformVector(conesBounds[i].n2,orientation)).subVector(0,1);
        conesBounds2D.push_back(tmp);
    }

    return verifySameHalfPlane(n1,n2,n3)&&verifyConesIntersection(c1,c2,c3,conesBounds2D);
}

bool iCub::grasp::isForceClosure(const iCub::grasp::ContactPoints &contactPoints)
{
    Vector diff1=contactPoints.c1-contactPoints.c2;
    Vector diff2=contactPoints.c1-contactPoints.c3;

    //Plane formed by the three contact points
    Vector plane=cross(diff2,diff1); plane/=norm(plane);

    double cos1=(dot(plane,contactPoints.n1));
    double cos2=(dot(plane,contactPoints.n2));
    double cos3=(dot(plane,contactPoints.n3));

    //The friction cone does not meet the plane on a plane
    if (!included(cos1,contactPoints.alpha1) || !included(cos2,contactPoints.alpha2) || !included(cos3,contactPoints.alpha3))
        return false;

    ConeBounds bounds1=findConeBounds(contactPoints.c1,contactPoints.n1,plane,contactPoints.alpha1);
    ConeBounds bounds2=findConeBounds(contactPoints.c2,contactPoints.n2,plane,contactPoints.alpha2);
    ConeBounds bounds3=findConeBounds(contactPoints.c3,contactPoints.n3,plane,contactPoints.alpha3);

    std::vector<ConeBounds> conesBounds;
    conesBounds.push_back(bounds1);
    conesBounds.push_back(bounds2);
    conesBounds.push_back(bounds3);

    return verifyCondition2D(contactPoints,plane,conesBounds);
}


