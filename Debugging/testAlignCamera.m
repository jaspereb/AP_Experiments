% Checks that alignCamera produces stationary results when run multiple
% times


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