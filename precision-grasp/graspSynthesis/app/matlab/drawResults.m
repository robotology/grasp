function drawResults(joints, ee, axisangle, center, dim, c1, c2, c3, n1, n2, n3, rotmat, hand, xdhat, odhat, cloud)

contactPoints=[c1' c2' c3'];
normals=[n1' n2' n3'];

x=[joints ee(1:3) axisangle];

drawResultsiCub(hand,x,center,contactPoints,normals);

for i=1:size(cloud,1)
   h=plot3(cloud(i,1),cloud(i,2),cloud(i,3),'*');
   set(h,'color',[cloud(i,4)/255 cloud(i,5)/255 cloud(i,6)/255]);
end
axis equal;

x1=[joints xdhat odhat];
drawResultsiCub(hand,x1,center,contactPoints,normals);

for i=1:size(cloud,1)
   h=plot3(cloud(i,1),cloud(i,2),cloud(i,3),'*');
   set(h,'color',[cloud(i,4)/255 cloud(i,5)/255 cloud(i,6)/255]);
end
axis equal;
