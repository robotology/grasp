%%Standard forward kinematics
function RT=FKinStd(DH,q,link,varargin)
    
    if nargin>0
        H0=varargin{1};
    else
        H0=eye(4,4);
    end

    H=cell(link+1,1);
    H{1}=H0;

    for i=2:link+1
        H{i}=H{i-1}*T(i-1,q(i-1,1),DH);
    end

    RT=H{size(H,1)};
end

function H=T(n,theta,P)
    
    theta=theta+P{n}.offset;
    c_theta=cos(theta);
    s_theta=sin(theta);
    c_alpha=cos(P{n}.alpha);
    s_alpha=sin(P{n}.alpha);

    H=[[c_theta -s_theta*c_alpha  s_theta*s_alpha P{n}.A*c_theta];...
       [s_theta  c_theta*c_alpha -c_theta*s_alpha P{n}.A*s_theta];...
       [      0          s_alpha          c_alpha         P{n}.D];...
       [      0                0                0              1]];
end

