function poses = getPosesStraight(cameraPose,expState)
% Get a set of poses in a straight line between the initial pose and final
% pose. Orientation will always point to the target

% Poses includes the start pose

% Caculate the distance between camera and target
startPose = cameraPose(1:3);
endPose = expState.grabPose(1:3);

poseDiff = endPose-startPose;
dist = sum(poseDiff.^2);
dist = sqrt(dist);

poseDiff = poseDiff./dist; %Unit vector in direction of travel

stepDist = dist/(expState.numPoses-1); %Lose one pose due to start step
poses(:,1) = cameraPose;

for n = 2:expState.numPoses
    pose = poses(:,n-1) + [stepDist*poseDiff;0;0;0;0]; %Assume fixed orientation
    poses(:,n) = pose;
end

assert(size(poses,1) == 7);
assert(size(poses,2) == expState.numPoses);

end