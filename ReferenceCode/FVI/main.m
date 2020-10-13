%This script simulates an active perception problem from Atanasov2014 where
%there are a series of fixed targets, a sensor model and some system
%control input model. This is specific to floating spheres as the targets
%and a robot arm as the sensor system.

%The sensor state consists of the {x,y,z,a,b,g} arm position where the
%angles are constrained to point towards the target

%The goal is to minimise the weighted target covariance at time T while
%ensuring the goal state is met at time T.

%Currently only for a single target

% Key Dimensions, n = numTargets:
% x 3nx1
% y 3nx1
% z 2nx1

clearvars
close all

% The FVI tree is implemented using http://tinevez.github.io/matlab-tree/
addpath('/home/jasper/jasperebrown@gmail.com/PhD/THEPIPELINE/VS_ActivePerception/3d-tree-matlab');

%% Setup parameters
x = [0,0,0,-pi/2,0,0]';
yTrue = [1.0, 2.0, 1.0]'; %True target position
yHat = yTrue + normrnd(0,3,3,1);
T = 15; %Max time steps
% plotState(x, yTrue, yTrue);

%% Build matrices
A = eye(3);
E = eye(3); %The initial sigma matrix, units in m^2??
W = eye(3); %Target noise covar, in m^2
V = 5*eye(2); %Observation noise covar, in pixels
%H is the linearised observation matrix wrt X which is then differentiated
%at the point yHat.


%% Run FVI
%From starting state until time = T, for each potential action
S0 = [{x},{E}];
FVI = tree(S0);
currentLeaves = 1;

for time = 1:T
    %For each current leaf, add children corresponding to actions. Also add
    %these to the 'newChildren' list
    childStates = {};
    childCovars = {};
    childParent = [];
    
    for leafIdx = currentLeaves
        node = FVI.get(leafIdx);
        x_t = node{1};
        actions = genReducedActions(x_t,yHat);
        
        for actId = 1:size(actions,2)
            action = actions{actId};
            
            x_tnext = x_t + [action;0;0;0];
            E_tnext = rand(3,3); %Dummy E
            
            childStates{end+1} = x_tnext;
            childCovars{end+1} = E_tnext;
            childParent(end+1) = leafIdx;
        end
    end
    
    %Check if any of the newly created children are redundant
    redundantIdx = [];
    for childIdx = 1:size(childStates,2)
        %Do any other children have the same state?
        for testIdx = 1:size(childStates,2)
            if(childIdx == testIdx)
                continue;
            end
            
            %Only check the xyz part of the state
            childState = childStates{childIdx};
            testState = childStates{testIdx};

            if(isequal(childState(1:3),testState(1:3))) %Same state
                if(log(det(childCovars{childIdx})) > log(det(childCovars{testIdx}))) %Redundant child
                    redundantIdx(end+1) = childIdx;
                end
            end
        end
    end
    
    %Remove redundant nodes
    childStates(redundantIdx) = [];
    childCovars(redundantIdx) = [];
    childParent(redundantIdx) = [];
    
    currentLeaves = [];
    %Add remaining ones to tree
    for addIdx = 1:size(childStates,2)
        [FVI, location] = FVI.addnode(childParent(addIdx), [{childStates{addIdx}},{childCovars{addIdx}}]);
        currentLeaves(end+1) = location; %Leaves to expand at next timestep
    end
    
    fprintf("Pruned %d nodes and tree is now %d at time %d \n", size(redundantIdx,2),size(FVI.Node,1),time);

    WHY ARE THESE NOT ADDING UP?
end







