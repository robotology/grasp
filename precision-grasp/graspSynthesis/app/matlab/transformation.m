function V=transformation(V,p,joints,i,finger,hand,rot_ee)
    pos0=fkiCub(finger,hand,[0 0 0]',0,rot_ee,i);
    [n,or0]=fkiCub(finger,hand,[0 0 0]',0,rot_ee,i+1);

    pos=fkiCub(finger,hand,joints',0,rot_ee,i);
    [n,or]=fkiCub(finger,hand,joints',0,rot_ee,i+1);

    rot_base=axis2dcm(or0);
    rot_base(1:3,4)=pos0;
    V=rototranslation(V,inv(rot_base),p);
    rotnew=axis2dcm(or);
    rotnew(1:3,4)=pos;
    V=rototranslation(V,rotnew,p);
end

