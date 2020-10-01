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

close all
clearvars
addpath('./triad');

% Simulation Parameters
expState.R = 0.00001;
expState.Qxy = 0.001;
expState.Qz = 0.001;
expState.Pxy = 0.05;
expState.Pz = 0.7;
expState.cameraParams = getCameraParams();
expState.initialPose = [0;0;0;1;0;0;0]; %(xyz position) and (wxyz quaternion)
expState.grabPose = [0;0;4;1;0;0;0]; %The final camera pose
expState.trellisDist = 4;
expState.targetPose = [0.5;0.7;expState.trellisDist]; % True target position
expState.targetZNoise = 0.25;
expState.numPoses = 11; %Number of poses to generate on each path, inc initial pose
expState.diagonalDist = 0.5; %Distance the diagonal path moves off straight

% Add Initial Noise
expState.targetPose(3) = expState.targetPose(3) + normrnd(0,expState.targetZNoise);
fprintf('Actual target pose is %f,%f,%f \n',expState.targetPose(1),expState.targetPose(2),expState.targetPose(3));

% Set up EKF for one target
A = eye(3); %Assumes fixed targets, only position matters
R = expState.R*eye(2);
Q = [expState.Qxy, 0, 0; 0, expState.Qxy, 0; 0, 0, expState.Qz];
P{1} = [expState.Pxy, 0, 0; 0, expState.Pxy, 0; 0, 0, expState.Pz];
functH = @ObsFunction;

fig_straight = figure(1);
fig_diag = figure(2);
fig_AP = figure(3);

cameraPose = expState.initialPose;
% plotState(cameraPose, expState, fig_straight);
[u,v] = getDetection(cameraPose, expState);

% Estimate initial state using first detection, assume that we know roughly 
% the distance to trellis
x(:,1) = estInitialState(u,v,cameraPose,expState);

%G, H are assumed identity
K = [];
[zHat,C{1}] = calcJac(functH, x(:,1), cameraPose, expState);

%Check calcJac has returned a correct observation
assert(zHat(1) == u);
assert(zHat(2) == v);

% Get poses list
straightPoses = getPosesStraight(cameraPose,expState);
diagonalPoses = getPosesDiagonal(cameraPose,expState);
APPoses = getPosesAP(cameraPose,expState);

expState.currExpName = 'Straight Path';
plotState(straightPoses, expState, fig_straight);
[x_straight,P_straight,z_straight] = runEKF(straightPoses,expState,functH,x,Q,R,A,K,C,P);

expState.currExpName = 'Diagonal Path';
plotState(diagonalPoses, expState, fig_diag);
[x_diag,P_diag,z_diag] = runEKF(diagonalPoses,expState,functH,x,Q,R,A,K,C,P);

% expState.currExpName = 'Active Perception Path';
% plotState(APPoses, expState, fig_AP);
% [x_AP,P_AP] = runEKF(APPoses,expState,functH,x,Q,R,A,K,C,P);

