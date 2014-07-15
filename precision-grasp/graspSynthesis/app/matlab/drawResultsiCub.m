function drawResultsiCub(hand, x, xo, contactPoints, normals)

    x1=contactPoints(:,1);
    x2=contactPoints(:,2);
    
    normals=normals/100.0;
    
    img=figure;
    
    hold on;
    plot3(x1(1),x1(2),x1(3),'*b','LineWidth',8.0);
    plot3([x1(1) x1(1)+normals(1,1)], [x1(2) x1(2)+normals(2,1)], [x1(3) x1(3)+normals(3,1)], 'r');
    plot3(x2(1),x2(2),x2(3),'*r','LineWidth',8.0);
    plot3([x2(1) x2(1)+normals(1,2)], [x2(2) x2(2)+normals(2,2)], [x2(3) x2(3)+normals(3,2)], 'r');
    
    if (size(contactPoints,2)>2)
        hold on;
        x3=contactPoints(:,3);
        plot3(x3(1),x3(2),x3(3),'*g','LineWidth',8.0);
        plot3([x3(1) x3(1)+normals(1,3)], [x3(2) x3(2)+normals(2,3)], [x3(3) x3(3)+normals(3,3)], 'r');
    end

    hold on;
    plot3(xo(1),xo(2),xo(3),'og');
    
    drawHandiCub(x,hand);
    axis equal;
    grid on;    
end