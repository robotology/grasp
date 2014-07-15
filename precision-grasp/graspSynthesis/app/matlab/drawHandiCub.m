function drawHandiCub(x,hand,varargin)
    
    if (size(varargin,2)>0)
        rotmat2=varargin{1};
    else
        rotmat2=eye(4,4);
    end;
    rotmat=axis2dcm(x(end-3:end));
    rotmat(1:3,4)=x(end-6:end-4);
    
    fkiCub(1,hand,x(1:3)',1,rotmat2*rotmat);
    fkiCub(2,hand,x(4:6)',1,rotmat2*rotmat);
    if (size(x,2)>13)
        fkiCub(3,hand,x(7:8)',1,rotmat2*rotmat);
        pos3link1=fkiCub(3,hand,x(7:8)',0,rotmat2*rotmat,0);
    else
        pos3link1=fkiCub(3,hand,[0 0]',0,rotmat2*rotmat,0);
    end
    ee=rotmat2*rotmat*[0 0 0 1]';
    hold on;
    plot3(ee(1),ee(2),ee(3),'*y','LineWidth',8.0);
    
    pos1link1=fkiCub(1,hand,x(1:3)',0,rotmat2*rotmat,1);
    pos2link1=fkiCub(2,hand,x(4:6)',0,rotmat2*rotmat,0);
    
    plot3([pos1link1(1) pos2link1(1)], [pos1link1(2) pos2link1(2)], [pos1link1(3) pos2link1(3)],'k');
    plot3([pos3link1(1) pos2link1(1)], [pos3link1(2) pos2link1(2)], [pos3link1(3) pos2link1(3)],'k');
    plot3([ee(1) pos1link1(1)], [ee(2) pos1link1(2)], [ee(3) pos1link1(3)],'k');
    plot3([ee(1) pos3link1(1)], [ee(2) pos3link1(2)], [ee(3) pos3link1(3)],'k');
    
    fill3([pos1link1(1) ee(1) pos2link1(1)], [pos1link1(2) ee(2) pos2link1(2)], [pos1link1(3) ee(3) pos2link1(3)], [0 0 0]);
    fill3([pos3link1(1) ee(1) pos2link1(1)], [pos3link1(2) ee(2) pos2link1(2)], [pos3link1(3) ee(3) pos2link1(3)], [0 0 0]);
    
    axis equal;
end