function  out=rototranslation(V,rot_tran,p)
    V=V';
    V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))];
    nv=rot_tran*V;
    set(p,'Vertices',nv(1:3,:)')
    out=nv(1:3,:)';
end

