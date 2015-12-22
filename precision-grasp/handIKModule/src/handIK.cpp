/* Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini
 * email:   ugo.pattacini@iit.it
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


#include <cmath>
#include <cstdio>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include "handIK.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class HandIK_NLP : public Ipopt::TNLP
{
protected:
    HandIK_Problem   &problem;
    HandIK_Variables  guess;
    HandIK_Variables  solution;

    Vector rpy,rpy_offs,encoders;
    Matrix Hc,thumbH,indexH,middleH;
    Matrix thumbH_deg0,thumbH_deg1,thumbH_deg2;
    Matrix Hc_thumbH_deg0,Hc_thumbH_deg1,Hc_thumbH_deg2;
    Vector thumbH_col2,thumbH_col3;
    Vector indexH_col2,indexH_col3;
    Vector middleH_col2,middleH_col3;

    Matrix dHc_dx0,dHc_dx1,dHc_dx2;
    Matrix dHc_dx3,dHc_dx4,dHc_dx5;
    Matrix Rz,Ry,Rx;
    Matrix dRz,dRy,dRx;

    /****************************************************************/
    void computeQuantities(const Ipopt::Number *x)
    {
        rpy[0]=x[3]+rpy_offs[0];
        rpy[1]=x[4]+rpy_offs[1];
        rpy[2]=x[5]+rpy_offs[2];
        Hc=rpy2dcm(rpy);
        Hc(0,3)=x[0];
        Hc(1,3)=x[1];
        Hc(2,3)=x[2];
        
        encoders[0]=x[9];
        encoders[1]=x[6];
        encoders[2]=x[7];
        encoders[3]=x[8];
        encoders[4]=x[10];
        encoders[5]=x[11];

        Vector joints;
        problem.thumb.getChainJoints(encoders,joints);
        thumbH=problem.thumb.getH(joints);
        thumbH_col2=thumbH.getCol(2);
        thumbH_col3=thumbH.getCol(3);
        thumbH_deg0=problem.thumb.getH(0,true);
        thumbH_deg1=problem.thumb.getH(2,true);
        thumbH_deg2=problem.thumb.getH(3,true);

        Hc_thumbH_deg0=Hc*thumbH_deg0;
        Hc_thumbH_deg1=Hc*thumbH_deg1;
        Hc_thumbH_deg2=Hc*thumbH_deg2;

        problem.index.getChainJoints(encoders,joints);
        indexH=problem.index.getH(joints);
        indexH_col2=indexH.getCol(2);
        indexH_col3=indexH.getCol(3);

        if (problem.nFingers==3)
        {
            encoders[6]=x[12];
            encoders[7]=x[13];

            problem.middle.getChainJoints(encoders,joints);
            middleH=problem.middle.getH(joints);
            middleH_col2=middleH.getCol(2);
            middleH_col3=middleH.getCol(3);
        }        

        double cr=cos(rpy[0]); double sr=sin(rpy[0]);
        double cp=cos(rpy[1]); double sp=sin(rpy[1]);
        double cy=cos(rpy[2]); double sy=sin(rpy[2]);

        Rz(0,0)=cy; Rz(1,1)=cy; Rz(0,1)=-sy; Rz(1,0)=sy;
        Ry(0,0)=cp; Ry(2,2)=cp; Ry(0,2)=-sp; Ry(2,0)=sp;
        Rx(1,1)=cr; Rx(2,2)=cr; Rx(1,2)=-sr; Rx(2,1)=sr;    

        // dHc/dx3 (roll)
        dRx(1,1)=-sr; dRx(2,2)=-sr; dRx(1,2)=-cr; dRx(2,1)=cr;
        dHc_dx3=Rz*Ry*dRx;

        // dHc/dx4 (pitch)
        dRy(0,0)=-sp; dRy(2,2)=-sp; dRy(0,2)=-cp; dRy(2,0)=cp;
        dHc_dx4=Rz*dRy*Rx;

        // dHc/dx5 (yaw)
        dRz(0,0)=-sy; dRz(1,1)=-sy; dRz(0,1)=-cy; dRz(1,0)=cy;
        dHc_dx5=dRz*Ry*Rx;
    }

public:
    /****************************************************************/
    HandIK_NLP(HandIK_Problem &p) : problem(p), solution(p.nFingers)
    {
        encoders.resize(9,0.0);
        rpy.resize(3,0.0);        

        dHc_dx0=zeros(4,4); dHc_dx0(0,3)=1.0;
        dHc_dx1=zeros(4,4); dHc_dx1(1,3)=1.0;
        dHc_dx2=zeros(4,4); dHc_dx2(2,3)=1.0;

        Rz=Ry=Rx=eye(4,4);
        dRz=dRy=dRx=zeros(4,4);

        // ee-rpy offset
        // they serve to change the frame
        // from the one relative to the object
        // to the one relative to the hand
        // when put palm down on top of the oject
        rpy_offs.resize(3,0.0);
        if (problem.hand=="right")
        {
            rpy_offs[0]=-M_PI;
            rpy_offs[1]=0.0;
            rpy_offs[2]=M_PI/2.0;
        }
        else
        {
            rpy_offs[0]=0.0;
            rpy_offs[1]=0.0;
            rpy_offs[2]=M_PI;
        }
    }

    /****************************************************************/
    virtual void setInitialGuess(const HandIK_Variables &g)
    {
        guess=g;
    }

    /****************************************************************/
    virtual HandIK_Variables getSolution() const
    {
        return solution;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=problem.nVars;
        m=3;//+3;
        nnz_jac_g=3+2*(6+3);//+7+8+9;
        nnz_h_lag=0;

        if (problem.nFingers==3)
        {
            m+=1;
            nnz_jac_g+=6+2;
        }
        
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        // retrieve the length of the middle finger to be used as relevant distance
        Vector zeros(9,0.0),joints;
        problem.middle.getChainJoints(zeros,joints);
        double d=1.5*norm(problem.middle.EndEffPosition(joints));

        // ee-xyz
        if (problem.hand=="right")
        {
            x_l[0]=0.0; x_u[0]=problem.dimensions[0]+d;
        }
        else
        {
            x_l[0]=-problem.dimensions[0]-d; x_u[0]=0.0;
        }
        x_l[1]=-problem.dimensions[1]-d; x_u[1]=0.0;//problem.dimensions[1]+d;
        x_l[2]=0.0;/*-problem.dimensions[2];*/   x_u[2]=problem.dimensions[2]+d;

        // ee-rpy
        if (problem.hand=="right")
        {
            x_l[3+0]=0.0;       x_u[3+0]=M_PI/2.0;
            x_l[3+1]=-M_PI/2.0; x_u[3+1]=0.0;
            x_l[3+2]=-M_PI/2.0; x_u[3+2]=M_PI/2.0;
        }
        else
        {
            x_l[3+0]=-M_PI/2.0; x_u[3+0]=0.0;
            x_l[3+1]=-M_PI/2.0; x_u[3+1]=0.0;
            x_l[3+2]=-M_PI/2.0; x_u[3+2]=M_PI/2.0;
        }

        // thumb joints
        iKinChain *chain=problem.thumb.asChain();
        x_l[6+0]=(*chain)[0].getMin(); x_u[6+0]=(*chain)[0].getMax();
        x_l[6+1]=(*chain)[2].getMin(); x_u[6+1]=(*chain)[2].getMax();
        x_l[6+2]=(*chain)[3].getMin(); x_u[6+2]=2.0*(*chain)[3].getMax();

        // index joints
        chain=problem.index.asChain();
        x_l[9+0]=(*chain)[0].getMin(); x_u[9+0]=3.0*(*chain)[0].getMax();
        x_l[9+1]=(*chain)[1].getMin(); x_u[9+1]=(*chain)[1].getMax();
        x_l[9+2]=(*chain)[2].getMin(); x_u[9+2]=2.0*(*chain)[2].getMax();

        // middle joints
        if (problem.nFingers==3)
        {
            chain=problem.middle.asChain();
            x_l[12+0]=(*chain)[0].getMin(); x_u[12+0]=(*chain)[0].getMax();
            x_l[12+1]=(*chain)[1].getMin(); x_u[12+1]=2.0*(*chain)[1].getMax();
        }        

        // ee-xyz out of the ellipsoid
        g_l[0]=1.0; g_u[0]=1e20;

        // contact points
        g_l[1]=g_u[1]=0.0;
        g_l[2]=g_u[2]=0.0;

        // thumb out of the ellipsoid
        /*g_l[3]=1.0; g_u[3]=1e20;
        g_l[4]=1.0; g_u[4]=1e20;
        g_l[5]=1.0; g_u[5]=1e20;*/

        // contact points (middle)
        if (problem.nFingers==3)
            g_l[3]=g_u[3]=0.0;

        return true;
    }
    
    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        // ee-xyz
        x[0]=guess.xyz_ee[0];
        x[1]=guess.xyz_ee[1];
        x[2]=guess.xyz_ee[2];

        // ee-rpy
        x[3+0]=guess.rpy_ee[0]-rpy_offs[0];
        x[3+1]=guess.rpy_ee[1]-rpy_offs[1];
        x[3+2]=guess.rpy_ee[2]-rpy_offs[2];

        // thumb joints
        x[6+0]=guess.joints[0];
        x[6+1]=guess.joints[1];
        x[6+2]=guess.joints[2];

        // index joints
        x[9+0]=guess.joints[3+0];
        x[9+1]=guess.joints[3+1];
        x[9+2]=guess.joints[3+2];

        // middle joints
        if (problem.nFingers==3)
        {
            x[12+0]=guess.joints[6+0];
            x[12+1]=guess.joints[6+1];
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x);

        obj_value=2.0+dot(problem.normalDirs[0],Hc*thumbH_col2)+
                      dot(problem.normalDirs[1],Hc*indexH_col2);

        //thumb joints far from their maximum
        /*iKinChain *chain=problem.thumb.asChain();
        double max_val=2.0*(*chain)[3].getMax();
        obj_value+=pow(exp(-(max_val-x[6+2])),10);

        //index joints far from their maximum
        chain=problem.index.asChain();
        max_val=2.0*(*chain)[2].getMax();
        obj_value+=pow(exp(-(max_val-x[9+2])),10);*/

        if (problem.nFingers==3)
        {
            obj_value+=1.0+dot(problem.normalDirs[2],Hc*middleH_col2);
            //middle joints far from their maximum
            /*chain=problem.middle.asChain();
            max_val=2.0*(*chain)[1].getMax();
            obj_value+=pow(exp(-(max_val-x[12+1])),10);*/
        }

        return true;
    }
    
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x);

        // ee-xyz
        // moving along x,y,z does not affect finger-tips normals
        grad_f[0]=0.0;
        grad_f[1]=0.0;
        grad_f[2]=0.0;

        // ee-rpy
        grad_f[3]=dot(problem.normalDirs[0],dHc_dx3*thumbH_col2)+
                  dot(problem.normalDirs[1],dHc_dx3*indexH_col2);

        grad_f[4]=dot(problem.normalDirs[0],dHc_dx4*thumbH_col2)+
                  dot(problem.normalDirs[1],dHc_dx4*indexH_col2);

        grad_f[5]=dot(problem.normalDirs[0],dHc_dx5*thumbH_col2)+
                  dot(problem.normalDirs[1],dHc_dx5*indexH_col2);

        if (problem.nFingers==3)
        {
            grad_f[3]+=dot(problem.normalDirs[2],dHc_dx3*middleH_col2);
            grad_f[4]+=dot(problem.normalDirs[2],dHc_dx4*middleH_col2);
            grad_f[5]+=dot(problem.normalDirs[2],dHc_dx5*middleH_col2);
        }

        // thumb
        Matrix thumbJ=problem.thumb.AnaJacobian(2).removeRows(4,2);
        thumbJ.setRow(3,Vector(thumbJ.cols(),0.0));
        grad_f[6]=dot(problem.normalDirs[0],Hc*thumbJ.getCol(0));
        grad_f[7]=dot(problem.normalDirs[0],Hc*thumbJ.getCol(1));
        grad_f[8]=dot(problem.normalDirs[0],Hc*thumbJ.getCol(2))/2.0+
                  dot(problem.normalDirs[0],Hc*thumbJ.getCol(3))/2.0;
        //thumb joints far from their maximum
        /*iKinChain *chain=problem.thumb.asChain();
        double max_val=2.0*(*chain)[3].getMax();
        grad_f[8]+=-10*pow(exp(-(max_val-x[6+2])),10);*/

        // index
        Matrix indexJ=problem.index.AnaJacobian(2).removeRows(4,2);
        indexJ.setRow(3,Vector(indexJ.cols(),0.0));
        grad_f[9] =dot(problem.normalDirs[1],Hc*indexJ.getCol(0))/3.0;
        grad_f[10]=dot(problem.normalDirs[1],Hc*indexJ.getCol(1));
        grad_f[11]=dot(problem.normalDirs[1],Hc*indexJ.getCol(2))/2.0+
                   dot(problem.normalDirs[1],Hc*indexJ.getCol(3))/2.0;
        //index joints far from their maximum
        /*chain=problem.index.asChain();
        max_val=2.0*(*chain)[2].getMax();
        grad_f[11]+=-10*pow(exp(-(max_val-x[9+2])),10);*/

        // middle
        if (problem.nFingers==3)
        {
            Matrix middleJ=problem.middle.AnaJacobian(2).removeRows(4,2);
            middleJ.setRow(3,Vector(middleJ.cols(),0.0));
            grad_f[12]=dot(problem.normalDirs[2],Hc*middleJ.getCol(0));
            grad_f[13]=dot(problem.normalDirs[2],Hc*middleJ.getCol(1))/2.0+
                       dot(problem.normalDirs[2],Hc*middleJ.getCol(2))/2.0;
            //middle joints far from their maximum
            /*chain=problem.middle.asChain();
            max_val=2.0*(*chain)[1].getMax();
            grad_f[13]+=-10*pow(exp(-(max_val-x[12+1])),10);*/
        }

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x);

        // ee-xyz out of the ellipsoid
        double tmp1,tmp2,tmp3;
        tmp1=x[0]/problem.dimensions[0];
        tmp2=x[1]/problem.dimensions[1];
        tmp3=x[2]/problem.dimensions[2];
        g[0]=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;

        // contact points
        g[1]=norm2(problem.contactPoints[0]-Hc*thumbH_col3);
        g[2]=norm2(problem.contactPoints[1]-Hc*indexH_col3);

        // thumb out of the ellipsoid
        /*tmp1=Hc_thumbH_deg0(0,3)/problem.dimensions[0];
        tmp2=Hc_thumbH_deg0(1,3)/problem.dimensions[1];
        tmp3=Hc_thumbH_deg0(2,3)/problem.dimensions[2];
        g[3]=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;

        tmp1=Hc_thumbH_deg1(0,3)/problem.dimensions[0];
        tmp2=Hc_thumbH_deg1(1,3)/problem.dimensions[1];
        tmp3=Hc_thumbH_deg1(2,3)/problem.dimensions[2];
        g[4]=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;

        tmp1=Hc_thumbH_deg2(0,3)/problem.dimensions[0];
        tmp2=Hc_thumbH_deg2(1,3)/problem.dimensions[1];
        tmp3=Hc_thumbH_deg2(2,3)/problem.dimensions[2];
        g[5]=tmp1*tmp1+tmp2*tmp2+tmp3*tmp3;*/

        // contact points (middle)
        if (problem.nFingers==3)
            g[3]=norm2(problem.contactPoints[2]-Hc*middleH_col3);
                
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        int cnt=0;

        if (values==NULL)
        {
            // return the structure of the Jacobian in triplet format

            // g[0] depends on ee-xyz
            iRow[cnt]=0; jCol[cnt]=0; cnt++;
            iRow[cnt]=0; jCol[cnt]=1; cnt++;
            iRow[cnt]=0; jCol[cnt]=2; cnt++;

            // g[1] depends on ee-xyz, ee-rpy
            for (int i=0; i<6; i++, cnt++)
            {
                iRow[cnt]=1; jCol[cnt]=i;
            }
            // g[1] depends on thumb
            iRow[cnt]=1; jCol[cnt]=6; cnt++;
            iRow[cnt]=1; jCol[cnt]=7; cnt++;
            iRow[cnt]=1; jCol[cnt]=8; cnt++;

            // g[2] depends on ee-xyz, ee-rpy
            for (int i=0; i<6; i++, cnt++)
            {
                iRow[cnt]=2; jCol[cnt]=i;
            }
            // g[2] depends on index
            iRow[cnt]=2; jCol[cnt]=9;  cnt++;
            iRow[cnt]=2; jCol[cnt]=10; cnt++;
            iRow[cnt]=2; jCol[cnt]=11; cnt++;

            // g[3] depends on ee-xyz, ee-rpy
            /*for (int i=0; i<6; i++, cnt++)
            {
                iRow[cnt]=3; jCol[cnt]=i;
            }
            // g[3] depends on thumb(0)
            iRow[cnt]=3; jCol[cnt]=6; cnt++;

            // g[4] depends on ee-xyz, ee-rpy
            for (int i=0; i<6; i++, cnt++)
            {
                iRow[cnt]=4; jCol[cnt]=i;
            }
            // g[4] depends on thumb(0:1)
            iRow[cnt]=4; jCol[cnt]=6; cnt++;
            iRow[cnt]=4; jCol[cnt]=7; cnt++;

            // g[5] depends on ee-xyz, ee-rpy
            for (int i=0; i<6; i++, cnt++)
            {
                iRow[cnt]=5; jCol[cnt]=i;
            }
            // g[5] depends on thumb(0:2)
            iRow[cnt]=5; jCol[cnt]=6; cnt++;
            iRow[cnt]=5; jCol[cnt]=7; cnt++;
            iRow[cnt]=5; jCol[cnt]=8; cnt++;*/

            if (problem.nFingers==3)
            {
                // g[6] depends on ee-xyz, ee-rpy
                for (int i=0; i<6; i++, cnt++)
                {
                    iRow[cnt]=3; jCol[cnt]=i;
                }
                // g[6] depends on middle
                iRow[cnt]=3; jCol[cnt]=12; cnt++;
                iRow[cnt]=3; jCol[cnt]=13; cnt++;
            }
        }
        else
        {
            computeQuantities(x);

            // return the values of the Jacobian of the constraints
            double dim0=(problem.dimensions[0]*problem.dimensions[0])/2.0;
            double dim1=(problem.dimensions[1]*problem.dimensions[1])/2.0;
            double dim2=(problem.dimensions[2]*problem.dimensions[2])/2.0;

            // dg[0]
            {
                values[0]=x[0]/dim0;
                values[1]=x[1]/dim1;
                values[2]=x[2]/dim2;
                cnt=3;
            }

            // dg[1]
            {
                Matrix J=problem.thumb.AnaJacobian(3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                Vector &p=thumbH_col3;
                Vector tmp=problem.contactPoints[0]-Hc*p;                

                values[cnt+0]=-2.0*dot(tmp,dHc_dx0*p);
                values[cnt+1]=-2.0*dot(tmp,dHc_dx1*p);
                values[cnt+2]=-2.0*dot(tmp,dHc_dx2*p);
                values[cnt+3]=-2.0*dot(tmp,dHc_dx3*p);
                values[cnt+4]=-2.0*dot(tmp,dHc_dx4*p);
                values[cnt+5]=-2.0*dot(tmp,dHc_dx5*p);
                values[cnt+6]=-2.0*dot(tmp,Hc*J.getCol(0));
                values[cnt+7]=-2.0*dot(tmp,Hc*J.getCol(1));
                values[cnt+8]=-2.0*dot(tmp,Hc*J.getCol(2))/2.0-dot(tmp,Hc*J.getCol(3))/2.0;
                cnt+=9;
            }

            // dg[2]
            {
                Matrix J=problem.index.AnaJacobian(3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                Vector &p=indexH_col3;
                Vector tmp=problem.contactPoints[1]-Hc*p;

                values[cnt+0]=-2.0*dot(tmp,dHc_dx0*p);
                values[cnt+1]=-2.0*dot(tmp,dHc_dx1*p);
                values[cnt+2]=-2.0*dot(tmp,dHc_dx2*p);
                values[cnt+3]=-2.0*dot(tmp,dHc_dx3*p);
                values[cnt+4]=-2.0*dot(tmp,dHc_dx4*p);
                values[cnt+5]=-2.0*dot(tmp,dHc_dx5*p);
                values[cnt+6]=-2.0*dot(tmp,Hc*J.getCol(0))/3.0;
                values[cnt+7]=-2.0*dot(tmp,Hc*J.getCol(1));
                values[cnt+8]=-2.0*dot(tmp,Hc*J.getCol(2))/2.0-dot(tmp,Hc*J.getCol(3))/2.0;
                cnt+=9;
            }

            // dg[3]
            /*{
                Matrix A0=dHc_dx0*thumbH_deg0;
                Matrix A1=dHc_dx1*thumbH_deg0;
                Matrix A2=dHc_dx2*thumbH_deg0;
                Matrix A3=dHc_dx3*thumbH_deg0;
                Matrix A4=dHc_dx4*thumbH_deg0;
                Matrix A5=dHc_dx5*thumbH_deg0;
                Matrix J=problem.thumb.AnaJacobian(0,3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                J=Hc*J;

                values[cnt+0]=Hc_thumbH_deg0(0,3)*A0(0,3)/dim0+Hc_thumbH_deg0(1,3)*A0(1,3)/dim1+Hc_thumbH_deg0(2,3)*A0(2,3)/dim2;
                values[cnt+1]=Hc_thumbH_deg0(0,3)*A1(0,3)/dim0+Hc_thumbH_deg0(1,3)*A1(1,3)/dim1+Hc_thumbH_deg0(2,3)*A1(2,3)/dim2;
                values[cnt+2]=Hc_thumbH_deg0(0,3)*A2(0,3)/dim0+Hc_thumbH_deg0(1,3)*A2(1,3)/dim1+Hc_thumbH_deg0(2,3)*A2(2,3)/dim2;
                values[cnt+3]=Hc_thumbH_deg0(0,3)*A3(0,3)/dim0+Hc_thumbH_deg0(1,3)*A3(1,3)/dim1+Hc_thumbH_deg0(2,3)*A3(2,3)/dim2;
                values[cnt+4]=Hc_thumbH_deg0(0,3)*A4(0,3)/dim0+Hc_thumbH_deg0(1,3)*A4(1,3)/dim1+Hc_thumbH_deg0(2,3)*A4(2,3)/dim2;
                values[cnt+5]=Hc_thumbH_deg0(0,3)*A5(0,3)/dim0+Hc_thumbH_deg0(1,3)*A5(1,3)/dim1+Hc_thumbH_deg0(2,3)*A5(2,3)/dim2;
                values[cnt+6]=Hc_thumbH_deg0(0,3)*J(0,0)/dim0 +Hc_thumbH_deg0(1,3)*J(1,0)/dim1 +Hc_thumbH_deg0(2,3)*J(2,0)/dim2;
                cnt+=7;
            }

            // dg[4]
            {
                Matrix A0=dHc_dx0*thumbH_deg1;
                Matrix A1=dHc_dx1*thumbH_deg1;
                Matrix A2=dHc_dx2*thumbH_deg1;
                Matrix A3=dHc_dx3*thumbH_deg1;
                Matrix A4=dHc_dx4*thumbH_deg1;
                Matrix A5=dHc_dx5*thumbH_deg1;
                Matrix J=problem.thumb.AnaJacobian(2,3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                J=Hc*J;

                values[cnt+0]=Hc_thumbH_deg1(0,3)*A0(0,3)/dim0+Hc_thumbH_deg1(1,3)*A0(1,3)/dim1+Hc_thumbH_deg1(2,3)*A0(2,3)/dim2;
                values[cnt+1]=Hc_thumbH_deg1(0,3)*A1(0,3)/dim0+Hc_thumbH_deg1(1,3)*A1(1,3)/dim1+Hc_thumbH_deg1(2,3)*A1(2,3)/dim2;
                values[cnt+2]=Hc_thumbH_deg1(0,3)*A2(0,3)/dim0+Hc_thumbH_deg1(1,3)*A2(1,3)/dim1+Hc_thumbH_deg1(2,3)*A2(2,3)/dim2;
                values[cnt+3]=Hc_thumbH_deg1(0,3)*A3(0,3)/dim0+Hc_thumbH_deg1(1,3)*A3(1,3)/dim1+Hc_thumbH_deg1(2,3)*A3(2,3)/dim2;
                values[cnt+4]=Hc_thumbH_deg1(0,3)*A4(0,3)/dim0+Hc_thumbH_deg1(1,3)*A4(1,3)/dim1+Hc_thumbH_deg1(2,3)*A4(2,3)/dim2;
                values[cnt+5]=Hc_thumbH_deg1(0,3)*A5(0,3)/dim0+Hc_thumbH_deg1(1,3)*A5(1,3)/dim1+Hc_thumbH_deg1(2,3)*A5(2,3)/dim2;
                values[cnt+6]=Hc_thumbH_deg1(0,3)*J(0,0)/dim0 +Hc_thumbH_deg1(1,3)*J(1,0)/dim1 +Hc_thumbH_deg1(2,3)*J(2,0)/dim2;
                values[cnt+7]=Hc_thumbH_deg1(0,3)*J(0,2)/dim0 +Hc_thumbH_deg1(1,3)*J(1,2)/dim1 +Hc_thumbH_deg1(2,3)*J(2,2)/dim2;
                cnt+=8;
            }

            // dg[5]
            {
                Matrix A0=dHc_dx0*thumbH_deg2;
                Matrix A1=dHc_dx1*thumbH_deg2;
                Matrix A2=dHc_dx2*thumbH_deg2;
                Matrix A3=dHc_dx3*thumbH_deg2;
                Matrix A4=dHc_dx4*thumbH_deg2;
                Matrix A5=dHc_dx5*thumbH_deg2;
                Matrix J=problem.thumb.AnaJacobian(3,3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                J=Hc*J;

                values[cnt+0]= Hc_thumbH_deg2(0,3)*A0(0,3)/dim0+Hc_thumbH_deg2(1,3)*A0(1,3)/dim1+Hc_thumbH_deg2(2,3)*A0(2,3)/dim2;
                values[cnt+1]= Hc_thumbH_deg2(0,3)*A1(0,3)/dim0+Hc_thumbH_deg2(1,3)*A1(1,3)/dim1+Hc_thumbH_deg2(2,3)*A1(2,3)/dim2;
                values[cnt+2]= Hc_thumbH_deg2(0,3)*A2(0,3)/dim0+Hc_thumbH_deg2(1,3)*A2(1,3)/dim1+Hc_thumbH_deg2(2,3)*A2(2,3)/dim2;
                values[cnt+3]= Hc_thumbH_deg2(0,3)*A3(0,3)/dim0+Hc_thumbH_deg2(1,3)*A3(1,3)/dim1+Hc_thumbH_deg2(2,3)*A3(2,3)/dim2;
                values[cnt+4]= Hc_thumbH_deg2(0,3)*A4(0,3)/dim0+Hc_thumbH_deg2(1,3)*A4(1,3)/dim1+Hc_thumbH_deg2(2,3)*A4(2,3)/dim2;
                values[cnt+5]= Hc_thumbH_deg2(0,3)*A5(0,3)/dim0+Hc_thumbH_deg2(1,3)*A5(1,3)/dim1+Hc_thumbH_deg2(2,3)*A5(2,3)/dim2;
                values[cnt+6]= Hc_thumbH_deg2(0,3)*J(0,0)/dim0 +Hc_thumbH_deg2(1,3)*J(1,0)/dim1 +Hc_thumbH_deg2(2,3)*J(2,0)/dim2;
                values[cnt+7]= Hc_thumbH_deg2(0,3)*J(0,2)/dim0 +Hc_thumbH_deg2(1,3)*J(1,2)/dim1 +Hc_thumbH_deg2(2,3)*J(2,2)/dim2;
                values[cnt+8]=(Hc_thumbH_deg2(0,3)*J(0,3)/dim0 +Hc_thumbH_deg2(1,3)*J(1,3)/dim1 +Hc_thumbH_deg2(2,3)*J(2,3)/dim2)/2.0;
                cnt+=9;
            }*/

            // dg[6]
            if (problem.nFingers==3)
            {
                Matrix J=problem.middle.AnaJacobian(3).removeRows(4,2);
                J.setRow(3,Vector(J.cols(),0.0));
                Vector &p=middleH_col3;
                Vector tmp=problem.contactPoints[2]-Hc*p;

                values[cnt+0]=-2.0*dot(tmp,dHc_dx0*p);
                values[cnt+1]=-2.0*dot(tmp,dHc_dx1*p);
                values[cnt+2]=-2.0*dot(tmp,dHc_dx2*p);
                values[cnt+3]=-2.0*dot(tmp,dHc_dx3*p);
                values[cnt+4]=-2.0*dot(tmp,dHc_dx4*p);
                values[cnt+5]=-2.0*dot(tmp,dHc_dx5*p);
                values[cnt+6]=-2.0*dot(tmp,Hc*J.getCol(0));
                values[cnt+7]=-2.0*dot(tmp,Hc*J.getCol(1))/2.0-dot(tmp,Hc*J.getCol(2))/2.0;
            }
        }
        
        return true;
    }

    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }

    /****************************************************************/
    bool get_scaling_parameters(Ipopt::Number &obj_scaling, bool &use_x_scaling,
                                Ipopt::Index n, Ipopt::Number *x_scaling,
                                bool &use_g_scaling, Ipopt::Index m,
                                Ipopt::Number *g_scaling)
    {
        obj_scaling=1.0;

        use_x_scaling=false;
        use_g_scaling=true;
        for (int i=0; i<m; i++)
            g_scaling[i]=1.0;

        return true;
    }
    
    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        // ee-xyz
        solution.xyz_ee[0]=x[0];
        solution.xyz_ee[1]=x[1];
        solution.xyz_ee[2]=x[2];

        // ee-rpy
        solution.rpy_ee[0]=x[3+0]+rpy_offs[0];
        solution.rpy_ee[1]=x[3+1]+rpy_offs[1];
        solution.rpy_ee[2]=x[3+2]+rpy_offs[2];

        // thumb joints
        solution.joints[0]=x[6+0];
        solution.joints[1]=x[6+1];
        solution.joints[2]=x[6+2];

        // index joints
        solution.joints[3+0]=x[9+0];
        solution.joints[3+1]=x[9+1];
        solution.joints[3+2]=x[9+2];

        // middle joints
        if (problem.nFingers==3)
        {
            solution.joints[6+0]=x[12+0];
            solution.joints[6+1]=x[12+1];
        }

        Ipopt::Number cost_fun;
        eval_f(n,x,true,cost_fun);
        solution.cost_fun=cost_fun;
    }
};


/****************************************************************/
void HandIK_Variables::print()
{
    printf("xyz_ee = (%s) [m]\n",xyz_ee.toString(3,3).c_str());
    printf("rpy_ee = (%s) [deg]\n",(CTRL_RAD2DEG*rpy_ee).toString(3,3).c_str());
    printf("joints = (%s) [deg]\n",(CTRL_RAD2DEG*joints).toString(3,3).c_str());
}


/****************************************************************/
bool HandIK_Solver::setInitialGuess(const HandIK_Variables &_guess)
{
    guess=_guess;
    return true;
}


/****************************************************************/
bool HandIK_Solver::solve(HandIK_Variables &solution)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-8);
    app->Options()->SetNumericValue("constr_viol_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",500);
    //app->Options()->SetStringValue("nlp_scaling_method","gradient-based");    
    //app->Options()->SetStringValue("nlp_scaling_method","none");
    app->Options()->SetStringValue("nlp_scaling_method","user-scaling");
    // #####################################
    //app->Options()->SetStringValue("jacobian_approximation","finite-difference-values");
    // #####################################
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    //app->Options()->SetIntegerValue("print_level",4);
    app->Options()->SetIntegerValue("print_level",0);
    app->Options()->SetStringValue("derivative_test","none");
    //app->Options()->SetStringValue("derivative_test","first-order");
    app->Initialize();

    Ipopt::SmartPtr<HandIK_NLP> nlp=new HandIK_NLP(problem);

    nlp->setInitialGuess(guess);
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    solution=nlp->getSolution();

    return (status==Ipopt::Solve_Succeeded);
}


