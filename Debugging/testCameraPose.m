% This script tests camera axes/TF defintions are consistent.

% cameraPose defines the transform from the world axes to the camera axes. 

clearvars
close all

% Parameters
expState = [];
expState.cameraParams = getCameraParams();
expState.currExpName = 'testCameraPose';
expState.targetPose = [-0.5;0.5;4.1]; % True target position 0,1.0,4.1
expState.grabPose = [expState.targetPose;1;0;0;0]; %The final camera pose
expState.minCamDistance = 0.25; %Below this z range, detections will not be generated
expState.maxCamDistance = inf;
expState.imageNoise = 0; %Sigma for pixel noise added to detections
expState.trellisDist = expState.targetPose(3);
functH = @ObsFunction;

% Generate a cameraPose
cameraPose = [1.0;0.2;0;1;0;0;0]; %(xyz position) and (wxyz quaternion)

% Align it to the target
alignedPose = alignCamera(cameraPose, expState.targetPose, expState);

% Display these to check generation, alignment and display of cameraPose
plotState(alignedPose, expState);

% Check converting axes
inCam = toCameraAxes(alignedPose,expState.targetPose);
assert(max(abs((fromCameraAxes(alignedPose,inCam')'-expState.targetPose)))<0.01);

% Project a point into the frame
[u,v] = getDetection(alignedPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, alignedPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 

% Check that aligncamera has worked
assert(abs(u-expState.cameraParams.Intrinsics.PrincipalPoint(1)) < 2);
assert(abs(v-expState.cameraParams.Intrinsics.PrincipalPoint(2)) < 2);

% Deproject a point from the frame
depth = toCameraAxes(alignedPose,expState.targetPose);
depth = depth(3); % True depth
estTarget = estInitialState(u,v,depth,alignedPose,expState);
assert(max(abs(estTarget' - expState.targetPose)) < 0.01);

disp("Camera alignment and deprojection tests passed");


% === To determine the orientation of pixel (u,v) axes ===
expState.targetPose = [0.0;0.0;4.0]; % True target position
expState.grabPose = [expState.targetPose;1;0;0;0]; %The final camera pose
cameraPose = [0;0;0;1;0;0;0]; %(xyz position) and (wxyz quaternion)

[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 
assert(abs(u-expState.cameraParams.Intrinsics.PrincipalPoint(1)) < 2);
assert(abs(v-expState.cameraParams.Intrinsics.PrincipalPoint(2)) < 2);

expState.targetPose = [1.0;0.0;4.0]; % True target position
[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 
assert(abs(v-expState.cameraParams.Intrinsics.PrincipalPoint(2)) < 2);
fprintf("When the target is moved in +x axis the u values is %d \n",u);

expState.targetPose = [-1.0;0.0;4.0]; % True target position
[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 
assert(abs(v-expState.cameraParams.Intrinsics.PrincipalPoint(2)) < 2);
fprintf("When the target is moved in -x axis the u values is %d \n",u);

expState.targetPose = [0.0;1.0;4.0]; % True target position
[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 
assert(abs(u-expState.cameraParams.Intrinsics.PrincipalPoint(1)) < 2);
fprintf("When the target is moved in +y axis the v values is %d \n",v);

expState.targetPose = [0.0;-1.0;4.0]; % True target position
[u,v] = getDetection(cameraPose, expState);
[zHat,~] = calcJac(functH, expState.targetPose, cameraPose, expState);
assert(abs(u-zHat(1)) < 2); % Check getDetection and ObsFunction agree within 2 pix
assert(abs(v-zHat(2)) < 2); 
assert(abs(u-expState.cameraParams.Intrinsics.PrincipalPoint(1)) < 2);
fprintf("When the target is moved in -y axis the v values is %d \n",v);
