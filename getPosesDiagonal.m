function poses = getPosesDiagonal(cameraPose,expState)
% Get a set of poses that run in a diagonal (triangle) from the start to 
% end pose. Orientation will always point to the target

% The diagonal part is offset in the x axis by diagonalDist, so the
% triangle is not quite isosceles but is close

% Poses includes the start pose and the end pose

%There must be a center pose, so numPoses must be odd
assert(mod(expState.numPoses,2) ~= 0); 

% Caculate the distance between camera and target
startPose = cameraPose(1:3);
endPose = expState.grabPose(1:3);

% %Midpoint calc works for camera starting at origin
% assert(startPose(1) == 0); 
% assert(startPose(2) == 0);

%Account for the diagonal offset distance
midPosex = (startPose(1)+endPose(1))/2 + expState.diagonalDist;
midPose = [midPosex;(startPose(2)+endPose(2))/2;(startPose(3)+endPose(3))/2];
dist = sqrt(sum((midPose-startPose).^2)) + sqrt(sum((endPose-midPose).^2));
stepDist = dist/(expState.numPoses-1); %Lose one pose due to start step

poses(:,1) = cameraPose;

%Path first half
poseDiff = (midPose-startPose)./(dist/2); %Unit vector in direction of travel
for n = 2:((expState.numPoses-1)/2)
    pose = poses(:,n-1) + [stepDist*poseDiff;0;0;0;0]; %Assume fixed orientation
    pose = alignCamera(pose,endPose,expState); %Get orientation
    poses(:,n) = pose;
end

%Add middle point
pose = [midPose;1;0;0;0];
pose = alignCamera(pose,endPose,expState); %Get orientation
poses(:,end+1) = pose;

%Path second half
poseDiff = (endPose-midPose)./(dist/2); %Unit vector in direction of travel
for n = (size(poses,2)+1):(expState.numPoses)
    pose = poses(:,n-1) + [stepDist*poseDiff;0;0;0;0]; %Assume fixed orientation
    pose = alignCamera(pose,endPose,expState); %Get orientation
    poses(:,n) = pose;
end

assert(size(poses,1) == 7);
assert(size(poses,2) == expState.numPoses);

end