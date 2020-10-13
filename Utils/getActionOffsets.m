function actionOffsets = getActionOffsets(expState)
%CALCACTIONDIST Calculate the x,y,z distance offsets that each action step 
% should apply using the number of total steps and the distance from the start to
% end pose. This ensures the actions form an overlapping grid. 

% The path always starts at the 0,0,0 pose and ends at expState.grabPose

pathVec = expState.grabPose(1:3)-expState.initialPose(1:3);
actionDist = norm(pathVec)/(expState.numPoses-1); %The grid spacing distance

% Construct vector orthogonal to pathVec and x axis
orthVec1 = cross(pathVec,[1;0;0]);

% Construct vector orthogonal to both of these
orthVec2 = cross(pathVec, orthVec1);

% Normalise
pathVec = pathVec./norm(pathVec);
orthVec1 = orthVec1./norm(orthVec1);
orthVec2 = orthVec2./norm(orthVec2);

assert(rank([pathVec, orthVec1, orthVec2]) == 3); %These should all be orthogonal

actionOffsets(1,:) = actionDist.*pathVec; %Forward
actionOffsets(2,:) = actionDist.*pathVec + actionDist.*orthVec1; %Up
actionOffsets(3,:) = actionDist.*pathVec - actionDist.*orthVec1; %Down
actionOffsets(4,:) = actionDist.*pathVec + actionDist.*orthVec2; %Right
actionOffsets(5,:) = actionDist.*pathVec - actionDist.*orthVec2; %Left

assert(size(actionOffsets,1) == 5);
assert(size(actionOffsets,2) == 3);

end

