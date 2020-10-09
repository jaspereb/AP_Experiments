% This script tests camera axes/TF defintions are consistent.

% cameraPose defines the transform from the world axes to the camera axes. 


expState = [];
expState.cameraParams = getCameraParams();
expState.currExpName = 'testCameraPose';
expState.targetPose = [0.0;1.0;4.1]; % True target position
expState.grabPose = [expState.targetPose;1;0;0;0]; %The final camera pose
expState.minCamDistance = 0.25; %Below this z range, detections will not be generated
expState.maxCamDistance = inf;
expState.imageNoise = 0; %Sigma for pixel noise added to detections
expState.trellisDist = expState.targetPose(3);
functH = @ObsFunction;

% Generate a cameraPose
cameraPose = [0;0;0;1;0;0;0]; %(xyz position) and (wxyz quaternion)

% Align it to the target
alignedPose = alignCamera(cameraPose, expState.targetPose, expState);

% Move the target somewhere new
newTarget = [0;0.1;4.1];

% Display these to check generation, alignment and display of cameraPose
plotState(alignedPose, expState);

% Project a point into the frame
[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2);
assert(abs(v-zHat(2)) < 2);

% Deproject a point from the frame
if(isequal(cameraPose(4:7),[1;0;0;0]))
    estTarget = estInitialState(u,v,cameraPose,expState);
    assert(max(abs(estTarget' - expState.targetPose)) < 0.01);
end

disp("Basic tests passed");

