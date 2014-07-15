function [x,o]=fkiCub(finger, hand, q1, draw, rotmat, varargin)

    dim=max(size(q1,1),size(q1,2));
    q=reshape(q1,dim,1);
    if finger==1
        H0=[0.478469 0.063689 -0.875792 -0.024029759;
            -0.878095 0.039246 -0.476873 -0.01193433;
            0.004 0.997198 0.074703 -0.00168926;
            0.0 0.0 0.0 1.0];
        color='b';
        active_joints=[1 0 1 1 1 1];
        l=size(q,1);
        qnew=zeros(l+1,1);
        qnew(1)=q(1);
        qnew(2)=0;
        for i=3:size(q)+1
            qnew(i)=q(i-1);
        end
        q=qnew;
    elseif finger==2
        H0=[0.898138 0.439714 0.0 0.00245549; 
            -0.43804 0.89472 0.087156 -0.025320433;
            0.038324 -0.078278 0.996195 -0.010973325;
            0.0 0.0 0.0 1.0];
        color='r';
        active_joints=[1 1 1 1 1];
        q(1)=q(1)/3;
    elseif finger==3
        H0=[1.0 0.0 0.0 0.0178;
            0.0 0.0 -1.0 -0.00830233;
            0.0 1.0 0.0 -0.0118;
            0.0 0.0 0.0 1.0];
        color='g';
        active_joints=[1 1 1 1];
    end

    if (hand=='l')
        H0(3,:)=-1*H0(3,:);
    end
    
    l=size(q,1);
    q(l)=q(l)/2;
    q(l+1)=q(l);
    
    if (size(varargin,2)==0)
        link=size(q,1);
    else 
        link=varargin{1};
    end

    DH=DHiCub(finger);
         
    Hroot=rotmat*FKinStd(DH,q,link,H0);

    x=Hroot(1:3,4);
    o=dcm2axis(Hroot(1:3,1:3));
    
    if (draw==1)
        hold on;
        view(3);
        grid;
        camproj orthographic
        rotate3d on

        iCubPlot(DH,q,color,rotmat,H0,active_joints);
        %iCubPlotCAD(DH,q,color,finger,rotmat,H0,active_joints);
    end

end
