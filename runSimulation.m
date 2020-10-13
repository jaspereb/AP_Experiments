% This script runs the experiments. It simulates a camera viewing one
% target. It calls the getPoses{straight, diagonal, AP} to get a poses list
% according to one of these methods. All information to be passed to this
% goes through the expState object. Each of these methods runs offline to
% generate a list of poses the camera should be at. 

% As the initial state, we assume the camera is automatically pointed to
% centre the target in the middle of the frame and the axes are defined so
% that z points from the camera to the target. This simplifies the problem
% definition without much loss of generality. 

% All units are meters, seconds, rads, pixels

% The camera pose is the transform from the world origin to the camera
% frame origin.

close all
clearvars
addpath('./Utils');

% Simulation Parameters
expState.R = 4;
expState.Qxy = 0.001;
expState.Qz = 0.001;
expState.Pxy = 0.05;
expState.Pz = 0.7;
expState.cameraParams = getCameraParams();
expState.trellisDist = 4;
expState.initialTargetPose = [0.0;0.0;expState.trellisDist]; % True target position
expState.grabPose = [expState.initialTargetPose;1;0;0;0]; %The final camera pose
expState.initialPose = [0;0;0;1;0;0;0]; %(xyz position) and (wxyz quaternion)
expState.initialPose = alignCamera(expState.initialPose, expState.grabPose(1:3),expState);
expState.targetZNoise = 0.05;
expState.targetXYNoise = 0.1;
expState.numPoses = 31; %Number of poses to generate on each path, inc initial pose. Must be odd
expState.diagonalDist = 0.5; %Distance the diagonal path moves off straight
expState.minCamDistance = 0.25; %Below this z range, detections will not be generated
expState.maxCamDistance = inf;
expState.imageNoise = 2; %Sigma for pixel noise added to detections
expState.showFigs = false; %Whether to show camera paths and state estimates for debugging
expState.plotResults = true; %Whether to plot experiment results (sigma over time)
expState.numRuns = 1000;
expState.printEKFStatus = false; %To print the result of each EKF run

% expState.targetPose(3) = expState.initialTargetPose(3) + 0.3; %For testing
fprintf('Initial target pose is %f,%f,%f \n',expState.initialTargetPose(1),expState.initialTargetPose(2),expState.initialTargetPose(3));

% Set up EKF for one target
A = eye(3); %Assumes fixed targets, only position matters
R = expState.R*eye(2);
Q = [expState.Qxy, 0, 0; 0, expState.Qxy, 0; 0, 0, expState.Qz];
P{1} = [expState.Pxy, 0, 0; 0, expState.Pxy, 0; 0, 0, expState.Pz];
functH = @ObsFunction;
cameraPose = expState.initialPose;

%G, H are assumed identity
K = [];

save('StartState.mat');

run Experiment1
% run Experiment2

tilefigs