function plotState(cameraPoses,expState,fig)
%Plotstate plots the camera pose and target pose. CameraPose is 7xN where 
%N is the number of poses to plot 

assert(size(cameraPoses,1) == 7);
figure(fig);
hold on

for col = 1:size(cameraPoses,2)
    pose = cameraPoses(:,col);
    R = quat2rotm(pose(4:7)');
    t = pose(1:3);
    pose3d = rigid3d(R',t');

    figure(fig);
    cam = plotCamera('AbsolutePose',pose3d,'Opacity',0, 'Size',0.1);
    grid on
    axis equal
    
    hold on
    
    %Plot Camera Axes
    ax = quat2axang(pose(4:7)');
    tf1 = makehgtform('translate',[t(1), t(2), t(3)]);
    tf2 = makehgtform('axisrotate',[ax(1),ax(2),ax(3)],ax(4));
    
    triad('Scale',0.2,'LineWidth',3,...
      'Tag','Triad Example','Matrix',...
      (tf1*tf2));
  
    
end

plot3(expState.targetPose(1),expState.targetPose(2),expState.targetPose(3), 'r*');
plot3(expState.grabPose(1),expState.grabPose(2),expState.grabPose(3), 'ro');

%Plot World Axes
tf = makehgtform;
triad('Scale',0.2,'LineWidth',3,...
          'Tag','Triad Example','Matrix',...
          tf);

%Set axis limits
axisBuffer = 0.5;
minx = min(min(cameraPoses(1,:)),expState.targetPose(1)) - axisBuffer;
maxx = max(max(cameraPoses(1,:)),expState.targetPose(1)) + axisBuffer;
miny = min(min(cameraPoses(2,:)),expState.targetPose(2)) - axisBuffer;
maxy = max(max(cameraPoses(2,:)),expState.targetPose(2)) + axisBuffer;
minz = min(min(cameraPoses(3,:)),expState.targetPose(3)) - axisBuffer;
maxz = max(max(cameraPoses(3,:)),expState.targetPose(3)) + axisBuffer;
xlim([minx,maxx]);
ylim([miny,maxy]);
zlim([minz,maxz]);
xlabel('x');
ylabel('y');
zlabel('z');
title(expState.currExpName);

end