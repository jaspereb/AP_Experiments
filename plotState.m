function plotState(cameraPose,expState)
%Plotstate plots the camera pose and target pose. CameraPose is 7xN where 
%N is the number of poses to plot 

assert(size(cameraPose,1) == 7);

for col = 1:size(cameraPose,2)
    pose = cameraPose(:,col);
    R = quat2rotm(pose(4:7)');
    t = pose(1:3);
    pose = rigid3d(R,t);

    cam = plotCamera('AbsolutePose',pose,'Opacity',0);
    grid on
    axis equal
    
    hold on

    
end

plot3(expState.targetPose(1),expState.targetPose(2),expState.targetPose(3), 'r*');

%Set axis limits


end