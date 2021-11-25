function hObject=fcn_QuadcopterAnimation(hObject, T, R, scale)
% R is the rotation from body frame to world frame
hAxis=gca;
if hObject==-1 % need to create a new object
    % entire object
    hObject = hgtransform('Parent',hAxis);
    %-----------------------------------------------------
    bodyColor=[0,128,128]/255;%[100,100,100]/255;%[244,164,96]/255;
    lightBlue=0.5*[70,130,180]/255;
    lightBrown=0.5*[244,164,96]/255;%[210,105,30]/255;
    edgeColor=0.4*[1,1,1];
    c1=0.8*[1,0,0]; % centers of the four blades
    c2=0.8*[0,1,0];
    c3=0.8*[-1,0,0];
    c4=0.8*[0,-1,0];
    % component: cylinder-blade1
    [x y z] = cylinder([1 1],30);
    x1=0.5*x+c1(1);     y1=0.5*y+c1(2);    z1=0.05*z+c1(3);
    x2=0.5*x+c2(1);     y2=0.5*y+c2(2);    z2=0.05*z+c2(3);
    x3=0.5*x+c3(1);     y3=0.5*y+c3(2);    z3=0.05*z+c3(3);
    x4=0.5*x+c4(1);     y4=0.5*y+c4(2);    z4=0.05*z+c4(3);
    hComBlade1=surface(x1,y1,z1,'FaceColor',lightBlue,'edgecolor', edgeColor);
    hComBlade2=surface(x2,y2,z2,'FaceColor',lightBlue,'edgecolor', edgeColor);
    hComBlade3=surface(x3,y3,z3,'FaceColor',lightBrown,'edgecolor', edgeColor);
    hComBlade4=surface(x4,y4,z4,'FaceColor',lightBrown,'edgecolor', edgeColor);
    set([hComBlade1,hComBlade2,hComBlade3,hComBlade4], 'parent', hObject);
    % component: cube-body
    lengthX=0.6; lengthY=0.6; lengthZ=0.3;
    [x,y,z]=fcn_GenerateBoard(lengthX,lengthZ,lengthY,3);
    z=z-0.05;
    hCubeBody=surface(x, y, z, 'facecolor', bodyColor,'edgecolor', edgeColor);
    set(hCubeBody, 'parent', hObject);
    % component: line - X-body frame
    hLine1=line([c1(1),c3(1)],[c1(2),c3(2)],[c1(3),c3(3)],'linewidth',2, 'color', bodyColor);
    hLine2=line([c2(1),c4(1)],[c2(2),c4(2)],[c2(3),c4(3)],'linewidth',2, 'color', bodyColor);
    set([hLine1,hLine2], 'parent', hObject);
   %-----------------------------------------------------
    % component: x-y-z local frame
   % hComLineX=line(2*[0 1], [0 0], [0 0], 'linewidth', 1, 'color', 'r');
   % hComLineY=line([0 0], 2*[0 1], [0 0], 'linewidth', 1, 'color', 'g');
   % hComLineZ=line([0 0], [0 0], 2*[0 1], 'linewidth', 1, 'color', 'b');
   % set(hComLineX, 'parent', hObject);
   % set(hComLineY, 'parent', hObject);
   % set(hComLineZ, 'parent', hObject);
    %-----------------------------------------------------
    % render
    set(gcf,'Renderer','zbuffer') % if Renderer is opengl, then you got a static movie!!!
%     drawnow
    % set the translation and orientation
    homoTrans=zeros(4);
    homoTrans(4,4)=1;
    homoTrans(1:3,1:3)=R'; % note here is transpose
    homoTrans(1:3,4)=T;
    homoTrans=homoTrans*makehgtform('scale',scale);
    set(hObject, 'Matrix', homoTrans)
else % just update the position and attitude of the object
    homoTrans=zeros(4);
    homoTrans(4,4)=1;
    homoTrans(1:3,1:3)=R'; % note here is transpose
    homoTrans(1:3,4)=T;
    homoTrans=homoTrans*makehgtform('scale',scale);
    set(hObject, 'Matrix', homoTrans)
    drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x, y, z]=fcn_GenerateBoard(length,width,height , n)
% the input is the 8 vertex of the board
xmin=-length/2;
xmax=length/2;
ymin=-height/2;
ymax=height/2;
zmin=-width/2;
zmax=width/2;
idx=1;
for xCoor=xmin:(xmax-xmin)/n:xmax
    x(1,idx)=0;
    x(2,idx)=xCoor;
    x(3,idx)=xCoor;
    x(4,idx)=0;
    y(1,idx)=0;
    y(2,idx)=ymax;
    y(3,idx)=ymax;
    y(4,idx)=0;
    z(1,idx)=zmax;
    z(2,idx)=zmax;
    z(3,idx)=zmin;
    z(4,idx)=zmin;
    idx=idx+1;
end
for xCoor=xmax:-(xmax-xmin)/n:xmin
    x(1,idx)=0;
    x(2,idx)=xCoor;
    x(3,idx)=xCoor;
    x(4,idx)=0;
    y(1,idx)=0;
    y(2,idx)=ymin;
    y(3,idx)=ymin;
    y(4,idx)=0;
    z(1,idx)=zmax;
    z(2,idx)=zmax;
    z(3,idx)=zmin;
    z(4,idx)=zmin;
    idx=idx+1;
end
x(1,idx)=0;
x(2,idx)=xmin;
x(3,idx)=xmin;
x(4,idx)=0;
y(1,idx)=0;
y(2,idx)=ymax;
y(3,idx)=ymax;
y(4,idx)=0;
z(1,idx)=zmax;
z(2,idx)=zmax;
z(3,idx)=zmin;
z(4,idx)=zmin;
