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

#include "psoThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::grasp;

PsoThread::PsoThread() : RateThread(20), cloud(new pcl::PointCloud<pcl::PointXYZ>),
    normals (new pcl::PointCloud <pcl::Normal>)
{
    done=true;
    work=false;
    init=true;
    set=false;
    fg=1e10;
    cost=0.06;
    /*float resolution = 128.0f;
    octree.setResolution(resolution);*/
}

bool PsoThread::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    particles=opt.check("particles",Value(20)).asInt();
    omega=opt.check("omega",Value(1.0)).asDouble();
    phi_p=opt.check("phi_p",Value(0.02)).asDouble();
    phi_g=opt.check("phi_g",Value(0.02)).asDouble();
    iterations=opt.check("iterations",Value(300)).asInt();
    limit_finger_max=opt.check("limit_finger_max",Value(0.08)).asDouble();
    limit_finger_min=opt.check("limit_finger_min",Value(0.02)).asDouble();
    hand_area=opt.check("hand_area",Value(0.00225)).asDouble();
    dimension=opt.check("dimension",Value(9)).asInt();
    vmax=opt.check("vmax",Value(0.3)).asDouble();
    vmin=opt.check("vmin",Value(-0.3)).asDouble();
    fp.resize(particles);
    nsg.resize(dimension);
    g.resize(dimension);

    this->start();
    return true;
}

void PsoThread::setData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::Normal>::Ptr normals, double alpha, int overlapping_cones)
{
    this->cloud->clear();
    this->normals->clear();
    copyPointCloud(*cloud,*this->cloud);
    copyPointCloud(*normals,*this->normals);
    this->overlapping_cones=overlapping_cones;
    this->alpha=alpha;
    work=true;
    done=false;
    this->resume();
    fg=1e10;
}

void PsoThread::run()
{
    if (work)
    {
        set=false;
        int seed=(int)Time::now()*1000;
        Random::seed(seed);
        kdtree.setInputCloud(cloud);
        //octree.addPointsFromInputCloud();
        visited.clear();

        if (cloud->size()<65)
            rand_method=0;
        else
            rand_method=2;
        rand_method=0;
        //rand_method=3;

        double time=Time::now();
        if (rand_method==0)
        {
            for (int i=0; i<cloud->size(); i++)
            {
                for (int j=i+1; j<cloud->size(); j++)
                {
                    for (int k=j+1; k<cloud->size(); k++)
                    {
                        yarp::sig::Vector ids(3); ids[0]=i; ids[1]=j; ids[2]=k;
                        yarp::sig::Vector p=assignPoints(ids);
                        yarp::sig::Vector n=assignNormals(ids);
                        ContactPoints triplet=fillContactPoints(p,n);
                        double f=fitness(triplet);
                        if (f<fg)
                        {
                            g=p;
                            nsg=n;
                            fg=f;
                            set=true;
                        }
                    }
                }
            }
            printf("Time exhaustive search %g\n", Time::now()-time);
            printf("best fg %g\n", fg);
        }
        else if (rand_method==3)
        {
            int iter=1;
            int iterationsRandom=6000;

            while (iter<=iterationsRandom && fg>cost)
            {
                yarp::sig::Vector ids(3);
                ids=findRandomId();
                yarp::sig::Vector p=assignPoints(ids);
                yarp::sig::Vector n=assignNormals(ids);
                ContactPoints triplet=fillContactPoints(p,n);
                double f=fitness(triplet);
                if (f<fg)
                {
                    g=p;
                    nsg=n;
                    fg=f;
                    set=true;
                }
                iter++;
            }
            if (fg<cost)
                printf("iteration %i\n", iter);
            printf("fg %g\n", fg);
        }
        else
        {
            if (rand_method==1)
            {
                for (int i=0; i<cloud->size(); i++)
                {
                    for (int j=i+1; j<cloud->size(); j++)
                    {
                        for (int k=j+1; k<cloud->size(); k++)
                        {
                            yarp::sig::Vector tmp(3);
                            tmp[0]=i; tmp[1]=j; tmp[2]=k;
                            visited.push_back(tmp);
                        }
                    }
                }
                printf("time for structure %d %g\n", overlapping_cones, Time::now()-time);
            }            
            double iter=1.0;
            double tot=iterations;
            double counter=iter;

            initialization();
            p=x;
            double time1=Time::now();
            while (iter<=iterations && fg>cost)
            {
                for (int i=0; i<particles; i++)
                {
                    yarp::sig::Vector ids(3,0.0);
                    if (!set)
                    {
                        if (rand_method==1)
                        {
                            int id=Random::uniform(0,visited.size()-1);
                            ids=visited[id];
                            visited.erase(visited.begin()+id);
                        }
                        else
                            ids=findRandomId();
                    }
                    else
                    {
                        for (int j=0; j<dimension; j++)
                        {
                            double rand_p=phi_p*Random::uniform();
                            double rand_g=phi_g*Random::uniform();
                            double velocity=omega*v.at(i)[j]+(rand_p*(p.at(i)[j]-x.at(i)[j]))+(rand_g*(g[j]-x.at(i)[j]));
                            if (velocity<vmin)
                                v.at(i)[j]=vmin;
                            else if (velocity>vmax)
                                v.at(i)[j]=vmax;
                            else
                                v.at(i)[j]=velocity;
                        }
                        x.at(i)+=v.at(i);
                        ids=findTriplet(x.at(i));
                    }
                
                    x.at(i)=assignPoints(ids);
                    ns.at(i)=assignNormals(ids);
                    ContactPoints triplet=fillContactPoints(x.at(i),ns.at(i));
                    double f=fitness(triplet);

                    if (f<fp[i])
                    {
                        p.at(i)=x.at(i);
                        fp[i]=f;
                    }

                    if (fp[i]<fg)
                    {
                        g=p.at(i);
                        nsg=ns.at(i);
                        fg=fp[i];
                        set=true;
                        printf("Time inizialization %d %g\n", overlapping_cones, Time::now()-time1);
                    }
                }
                if (set)
                    omega=(1-(counter/tot));
                else
                    omega=0.8;

                iter+=1.0;
                counter+=1.0;            

                if ((int)iter%50==0 && set)
                {
                    double distances;
                    for (int j=0; j<x.size(); j++)
                        distances+=norm(x.at(j)-g);
                    distances/=x.size();

                    if (distances<0.2)
                    {
                        tot=iterations-iter+1;
                        counter=1;
                        omega=1;
                        initialization();
                    }
                }
            }
            printf("Time pso %d %g\n", overlapping_cones, Time::now()-time1);
            printf("iteration %g\n", iter);
            printf("fg %g\n", fg);
        }
        done=true;
        work=false;
    }  
    this->suspend();
}

yarp::sig::Vector PsoThread::findRandomId()
{
    yarp::sig::Vector ids(3);
    bool assigned=false;
    while(!assigned)
    {
        yarp::sig::Vector tmp(3);
        tmp[0]=Random::uniform(0,cloud->size()-1);
        tmp[1]=Random::uniform(0,cloud->size()-2);
        tmp[2]=Random::uniform(0,cloud->size()-3);
        ids=findDifferentIds(tmp);              
        if (visited.size()==0)
        {
            visited.push_back(ids);
            assigned=true;
        }
        else
        {
            bool equal=false;
            for (std::vector<yarp::sig::Vector>::iterator it=visited.begin(); it!=visited.end(); it++)
            {                                
                if ((*it)[0]==ids[0] && (*it)[1]==ids[1] && (*it)[2]==ids[2])
                {
                    equal=true;
                    break;
                }
            }
            if (!equal)
            {
                assigned=true;
                visited.push_back(ids);
            }
        }
    }
    return ids;
}

void PsoThread::initialization()
{
    yarp::sig::Vector idx(3);
    fp=1e20;
    for (int i=0; i<particles; i++)
    {
        if (rand_method==1)
        {
            int id=Random::uniform(0,visited.size()-1);
            idx=visited[id];
            visited.erase(visited.begin()+id);
        }
        else
            idx=findRandomId();

        if (init)
        {
            x.push_back(assignPoints(idx));
            p.push_back(assignPoints(idx));
            ns.push_back(assignNormals(idx));
            yarp::sig::Vector tmp(dimension);
            for (int j=0; j<tmp.size(); j++)
                tmp[j]=fmod(Random::uniform(),vmax)*pow(-1.0,Random::uniform(1,2));
            v.push_back(tmp);
        }
        else
        {
            x.at(i)=assignPoints(idx);
            ns.at(i)=assignNormals(idx);
            for (int j=0; j<dimension; j++)
                v.at(i)[j]=fmod(Random::uniform(),vmax)*pow(-1.0,Random::uniform(1,2));
        }
    }
    init=false;
}

bool PsoThread::isSuccessful()
{
    return set;
}

ContactPoints PsoThread::getBestTriplet()
{
    ContactPoints triplet=fillContactPoints(g,nsg);
    bool a=isForceClosure(triplet);
    printf("fc %d %s\n", overlapping_cones, a?"yes":"no");
   
    int n_cones=3;
    yarp::sig::Vector angles(3);
    yarp::sig::Vector idx(3);
    if (!counterOverlap(triplet.n1,triplet.n2,angles[0]))
        n_cones-=1;
    if (!counterOverlap(triplet.n1,triplet.n3,angles[1]))
        n_cones-=1;
    if (!counterOverlap(triplet.n2,triplet.n3,angles[2]))
        n_cones-=1;
    printf("ov cones %d %d\n", overlapping_cones, n_cones);
    triplet.ov_cones=n_cones;

    yarp::sig::Vector n1=triplet.n1/norm(triplet.n1)*0.003;
    yarp::sig::Vector n2=triplet.n2/norm(triplet.n2)*0.003;
    yarp::sig::Vector n3=triplet.n3/norm(triplet.n3)*0.003;

    triplet.c1-=n1;
    triplet.c2-=n2;
    triplet.c3-=n3;

    return triplet;
}

double PsoThread::getCost()
{
    return fg;
}

ContactPoints PsoThread::fillContactPoints(yarp::sig::Vector &xi, yarp::sig::Vector &nsi)
{
    yarp::sig::Vector c1(3),c2(3),c3(3),n1(3),n2(3),n3(3);
    ContactPoints triplet;
    c1[0]=xi[0];
    c1[1]=xi[1];
    c1[2]=xi[2];
    c2[0]=xi[3];
    c2[1]=xi[4];
    c2[2]=xi[5];
    c3[0]=xi[6];
    c3[1]=xi[7];
    c3[2]=xi[8];

    n1[0]=nsi[0];
    n1[1]=nsi[1];
    n1[2]=nsi[2];
    n2[0]=nsi[3];
    n2[1]=nsi[4];
    n2[2]=nsi[5];
    n3[0]=nsi[6];
    n3[1]=nsi[7];
    n3[2]=nsi[8];

    triplet.c1=c1;
    triplet.c2=c2;
    triplet.c3=c3;
    triplet.n1=n1;
    triplet.n2=n2;
    triplet.n3=n3;
    triplet.alpha1=alpha;
    triplet.alpha2=alpha;
    triplet.alpha3=alpha;

    return triplet;
}

double PsoThread::fitness(ContactPoints &triplet)
{
    double f=0.0;
    double dist1=norm(triplet.c1-triplet.c2);
    double dist2=norm(triplet.c1-triplet.c3);
    double dist3=norm(triplet.c2-triplet.c3);

    if (dist1>limit_finger_max || dist2>limit_finger_max || dist3>limit_finger_max)
        return 1e10;
    if (dist1<0.01 || dist2<0.01 || dist3<0.01)
        return 1e10;
    if (dist1>limit_finger_min && dist2>limit_finger_min && dist3>limit_finger_min)
        return 1e10;

    if (!isForceClosure(triplet))
        return 1e10;

    int n_cones=3;
    yarp::sig::Vector angles(3);
    yarp::sig::Vector idx(3);
    if (!counterOverlap(triplet.n1,triplet.n2,angles[0]))
    {
        idx[0]=0;
        n_cones-=1;
    }
    else
        idx[0]=1;
    if (!counterOverlap(triplet.n1,triplet.n3,angles[1]))
    {
        idx[1]=0;
        n_cones-=1;
    }
    else
        idx[1]=1;
    if (!counterOverlap(triplet.n2,triplet.n3,angles[2]))
    {
        idx[2]=0;
        n_cones-=1;
    }
    else
        idx[2]=1;

    if (n_cones!=overlapping_cones)
        f+=penaltyCones(n_cones,idx,angles);

    double diff=(double)(abs(overlapping_cones-n_cones));
    double w=1e-01*(pow(2,diff)+1);
    double current_area=triangleArea(triplet.c1,triplet.c2,triplet.c3);

    //the penalty increases like a line
    if (current_area>hand_area)
        f+=1e03*current_area;
    else
    {
        //the penalty decreases like a paraboloid
        double diff_area2=(current_area-hand_area)*(current_area-hand_area);
        double diff_area4=diff_area2*diff_area2;
        f+=w*1e11*abs(diff_area4);
    }

    return f;
}

double PsoThread::penaltyCones(int n_cones, yarp::sig::Vector &idx, yarp::sig::Vector &angles)
{
    double f=0.0;
    int diff=overlapping_cones-n_cones;
    int test;
    if (diff>0)
        test=0;
    else
        test=1;
    //if the number of wanted overlapping_cones is very different from the number of
    //effectively overlapping cones, then the angles summed to the penalty function are
    //weighted more
    double w=abs(diff*diff*diff);
    std::vector<double> missingAngles;
    for (int i=0; i<idx.size(); i++)
    {
        if (idx[i]==test)
            missingAngles.push_back(angles[i]);
    }
    std::sort(missingAngles.begin(), missingAngles.end());
    for (int i=0; i<abs(diff); i++)
        f+=(w*missingAngles[i]);
    return f;
}

double PsoThread::triangleArea(yarp::sig::Vector &p1, yarp::sig::Vector &p2, yarp::sig::Vector &p3)
{
    return norm(cross(p2-p1,p3-p1))/2;
}

bool PsoThread::counterOverlap(yarp::sig::Vector &n1, yarp::sig::Vector &n2, double &angle)
{
    n1=n1/norm(n1);
    n2=n2/norm(n2);

    double theta=abs(M_PI-acos(dot(n1,n2)));
    if (theta<2*this->alpha)
    {
        angle=(2*this->alpha)-theta;
        return true;
    }
    else
    {
        angle=theta-(2*this->alpha);
        return false;
    }
}

yarp::sig::Vector PsoThread::assignPoints(const yarp::sig::Vector &ids)
{
    yarp::sig::Vector res(dimension);
    res[0]=cloud->at(ids[0]).x;
    res[1]=cloud->at(ids[0]).y;
    res[2]=cloud->at(ids[0]).z;
    res[3]=cloud->at(ids[1]).x;
    res[4]=cloud->at(ids[1]).y;
    res[5]=cloud->at(ids[1]).z;
    res[6]=cloud->at(ids[2]).x;
    res[7]=cloud->at(ids[2]).y;
    res[8]=cloud->at(ids[2]).z;
    return res;
}

yarp::sig::Vector PsoThread::assignNormals(const yarp::sig::Vector &ids)
{
    yarp::sig::Vector res(dimension);
    res[0]=normals->at(ids[0]).normal_x;
    res[1]=normals->at(ids[0]).normal_y;
    res[2]=normals->at(ids[0]).normal_z;
    res[3]=normals->at(ids[1]).normal_x;
    res[4]=normals->at(ids[1]).normal_y;
    res[5]=normals->at(ids[1]).normal_z;
    res[6]=normals->at(ids[2]).normal_x;
    res[7]=normals->at(ids[2]).normal_y;
    res[8]=normals->at(ids[2]).normal_z;
    return res;
}

int PsoThread::findClosestPoint(pcl::PointXYZ &point)
{
    int k=1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    int id;
    if (kdtree.nearestKSearch (point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        id=pointIdxNKNSearch[0];
    else
        id=-1;
    return id;
}

yarp::sig::Vector PsoThread::findTriplet(const yarp::sig::Vector &vect)
{
    pcl::PointXYZ p1(vect[0],vect[1],vect[2]);
    pcl::PointXYZ p2(vect[3],vect[4],vect[5]);
    pcl::PointXYZ p3(vect[6],vect[7],vect[8]);

    yarp::sig::Vector ids(3);

    ids[0]=findClosestPoint(p1);
    ids[1]=findClosestPoint(p2);
    ids[2]=findClosestPoint(p3);

    if (ids[0]!=ids[1] && ids[0]!=ids[2] && ids[1]!=ids[2])
        return ids;
    else
    {
        if (rand_method==1)
        {
            int id=Random::uniform(0,visited.size()-1);
            ids=visited[id];
            visited.erase(visited.begin()+id);
        }
        else
        {
            yarp::sig::Vector tmp(3);
            tmp[0]=Random::uniform(0,cloud->size()-1);
            tmp[1]=Random::uniform(0,cloud->size()-2);
            tmp[2]=Random::uniform(0,cloud->size()-3);
            ids=findDifferentIds(tmp);    
        }
    }
    return ids;
}

yarp::sig::Vector PsoThread::findDifferentIds(const yarp::sig::Vector &ids)
{
    yarp::sig::Vector idsNew(3);
    idsNew[0]=ids[0];
    double small=0.0;
    double big=0.0;

    if (ids[1]>=ids[0])
    {
        idsNew[1]=ids[1]+1;
        big=ids[1]+1;
        small=ids[0];
    }
    else
    {
        idsNew[1]=ids[1];
        big=ids[0];
        small=ids[1];
    }

    if (ids[2]<small)
        idsNew[2]=ids[2];
    else if (ids[2]>=small)
        idsNew[2]=ids[2]+1;
    if (idsNew[2]>=big)
        idsNew[2]=idsNew[2]+1;

    return idsNew;
}

bool PsoThread::checkDone()
{
    return done;
}

void PsoThread::close()
{
 
}

void PsoThread::threadRelease() 
{

}

