function drawHandiCubCAD(x,hand)

    Vt=cell(4,1); pt=cell(4,1);
    Vind=cell(4,1); pind=cell(4,1);
    Vm=cell(3,1); pm=cell(3,1);

    %load pieces and make a base rototranslation between
    %CAD reference frame and root reference frame
    if (hand=='r')
        load right_hand;

        rotx90=[1 0 0 -pi/2];
        rot=axis2dcm(rotx90);
        rotz90=[0 0 1 pi/2];
        rot=axis2dcm(rotz90)*rot;
        rot(2,4)=-0.01;        
    else
        load left_hand;

        rotx90=[1 0 0 pi/2];
        rot=axis2dcm(rotx90);
        rotz90=[0 0 1 pi/2];
        rot=axis2dcm(rotz90)*rot;
        rot(2,4)=-0.01;        
    end

    ppalm=save_CAD(Fpalm,Vpalm,'g');
    Vt{1}=Vt1; Vt{2}=Vt2; Vt{3}=Vt3; Vt{4}=Vt4;
    pt{1}=save_CAD(Ft1,Vt{1},'b');
    pt{2}=save_CAD(Ft2,Vt{2},[0.4,0.4,0.4]);
    pt{3}=save_CAD(Ft3,Vt{3},[0.4,0.4,0.4]);
    pt{4}=save_CAD(Ft4,Vt{4},'k');
    Vind{1}=Vi1; Vind{2}=Vi2; Vind{3}=Vi3; Vind{4}=Vi4;
    pind{1}=save_CAD(Fi1,Vind{1},'b');
    pind{2}=save_CAD(Fi2,Vind{2},[0.4,0.4,0.4]);
    pind{3}=save_CAD(Fi3,Vind{3},[0.4,0.4,0.4]);
    pind{4}=save_CAD(Fi4,Vind{4},'k');
    Vm{1}=Vm1; Vm{2}=Vm2; Vm{3}=Vm3;
    pm{1}=save_CAD(Fm1,Vm{1},'b');
    pm{2}=save_CAD(Fm2,Vm{2},[0.4,0.4,0.4]);
    pm{3}=save_CAD(Fm3,Vm{3},'k');
    pl=save_CAD(Fl,Vl,[0.4,0.4,0.4]);
 
    %rototranslation matrix for end effector
    rot_ee=axis2dcm(x(12:15));
    rot_ee(1:3,4)=x(9:11);
    
    rot=rot_ee*rot;
    
    rototranslation(Vpalm,rot,ppalm);
    for i=1:size(Vt,1)
       Vt{i}=rototranslation(Vt{i},rot,pt{i}); 
    end
    for i=1:size(Vind,1)
       Vind{i}=rototranslation(Vind{i},rot,pind{i}); 
    end
    for i=1:size(Vm,1)
       Vm{i}=rototranslation(Vm{i},rot,pm{i}); 
    end
    rototranslation(Vl,rot,pl);
         
    active_joints_thumb=[0 2 3 4];
    for i=1:size(active_joints_thumb,2)
        transformation(Vt{i},pt{i},x(1:3),active_joints_thumb(i),1,hand,rot_ee);
    end
    
    active_joints_ind=[0 1 2 3];
    for i=1:size(active_joints_ind,2)
       transformation(Vind{i},pind{i},x(4:6),active_joints_ind(i),2,hand,rot_ee); 
    end
    
    active_joints_middle=[0 1 2];
    for i=1:size(active_joints_middle,2)
       transformation(Vm{i},pm{i},x(7:8),active_joints_middle(i),3,hand,rot_ee); 
    end

    light                               
    daspect([1 1 1])
    view(3)
    xlabel('X'),ylabel('Y'),zlabel('Z')

end

