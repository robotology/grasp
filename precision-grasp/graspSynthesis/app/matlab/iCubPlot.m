function Tr=iCubPlot(DH,q,color,varargin)
Twb=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

if(size(varargin,2)==1)
    Twb=varargin{1};
    active_joints=ones(size(q));
end
if(size(varargin,2)==2)
    Twb=varargin{1};
    H0=varargin{2};
    active_joints=ones(size(q));
end
if (size(varargin,2)==3)
    Twb=varargin{1};
    H0=varargin{2};
    active_joints=varargin{3};
end

DHmat=zeros(size(DH,1),size(DH,2));
for i=1:size(DH,2)
    DHmat(i,1)=DH{i}.alpha;
    DHmat(i,2)=DH{i}.A;
    DHmat(i,3)=DH{i}.D;
    DHmat(i,4)=DH{i}.offset;
end

hold on
magnitude=max(max(DHmat(:,2:3))); %this will set the scale for the initial frame and the joints axis display
xlabel('x');
ylabel('y');
zlabel('z');

%% plotting the initial frame

init_frame=[magnitude/5 magnitude/5 magnitude/5 1 ; 0 0 0 1];
%plot3(init_frame(:,1),[0;0],[0;0],'r','LineWidth',3)
%plot3([0;0],init_frame(:,2),[0;0],'g','LineWidth',3)
%plot3([0;0],[0;0],init_frame(:,3),'b','LineWidth',3)

%% plotting the first joint in the z direction (DH rule)

init_joint=[0 0 0 1 ;0 0 magnitude/5 1];

%Ts1=FKinematics(DH,base,1);
%Ts1=eye(4);
for i=0:size(DHmat,1)
    Tr=Twb*FKinStd(DH,q,i,H0);
    position=Tr*[0 0 0 1]';
    x(1,i+1)=position(1);
    y(1,i+1)=position(2);
    z(1,i+1)=position(3);
    if (active_joints(i+1)==1)
        result=Tr*init_joint';
        joint=result';
        plot3(joint(:,1),joint(:,2),joint(:,3),'m:','LineWidth',3)
        plot3(x(1,i+1),y(1,i+1),z(1,i+1),'ok','MarkerFaceColor','k')
    end
end


plot3(x,y,z,color,'LineWidth',2)
%plot3(x(base),y(base),z(base),'go','LineWidth',2);

%plot3(x(1,size(DH,1)),y(1,size(DH,1)),z(1,size(DH,1)),'ok','MarkerFaceColor','k')

hold off


end