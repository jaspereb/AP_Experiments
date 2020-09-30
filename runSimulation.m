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

% Simulation Parameters
expState.R = 1;
expState.Qxy = 0.001;
expState.Qz = 0.001;
expState.Pxy = 0.05;
expState.Pz = 0.7;
expState.intrinsics = getD435Intrinsics();
expState.initialPose = [0,0,0,1,0,0,0]'; %(xyz position) and (wxyz quaternion)
expState.targetPose = [0;0;4]; % True target position

% Set up EKF for one target
A = eye(3); %Assumes fixed targets, only position matters
R = expState.R*eye(2);
Q = [expState.Qxy, 0, 0; 0, expState.Qxy, 0; 0, 0, expState.Qz];
P{1} = [expState.Pxy, 0, 0; 0, expState.Pxy, 0; 0, 0, expState.Pz];

cameraPose = expState.initialPose;
[u,v] = getDetection(cameraPose, expState);

x(:,1) = ; % Estimate initial state using first detection

functH = @ObsFunction;

%G, H are assumed identity
K = [];
Y = [];
[z,C{1}] = calcJac(functH, x(:,1), camPosInv{1}, camRotInv{1}, expState.intrinsics);

for time = 2:size(observations,2)
    %Predict Stage
    x(:,time) = A*x(:,time-1);
    xPrior(:,time) = x(:,time);
    P{time} = A*P{time-1}*A' + Q;
    
    [z,C{time}] = calcJac(functH, x(:,time), camPosInv{time}, camRotInv{time}, intrinsics);
    
    K{time} = P{time}*C{time}'*inv(R + C{time}*P{time}*C{time}');
    x(:,time) = x(:,time) + K{time}*(observations(:,time) - z);
    innov(:,time) = observations(:,time) - z;
    P{time} = P{time} - K{time}*C{time}*P{time};
end

% Get poses list
straightPoses = getPosesStraight();
diagonalPoses = getPosesDiagonal();
APPoses = getPosesAP();

% Run EKF update on position


% Get new position


