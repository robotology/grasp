function v=dcm2axis(R)

sz=size(R);

if (sz(1)<3) || (sz(2)<3)
    disp('failed');
    v=[];
    return;
end

v=zeros(4,1);
v(1)=R(3,2)-R(2,3);
v(2)=R(1,3)-R(3,1);
v(3)=R(2,1)-R(1,2);
r=norm(v);
theta=atan2(0.5*r,0.5*(R(1,1)+R(2,2)+R(3,3)-1));

if (r<1e-9)
    A=R(1:3,1:3);
    
    [~,~,V]=svd(A-eye(3,3));

    v(1)=V(1,3);
    v(2)=V(2,3);
    v(3)=V(3,3);
    r=norm(v);
end

v=v/r;
v(4)=theta;
