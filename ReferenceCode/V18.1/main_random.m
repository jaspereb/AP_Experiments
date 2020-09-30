%Choose random actions to take

%This script simulates an active perception problem from Atanasov2019 where
%there are a series of fixed targets, a sensor model and some system
%control input model. This is specific to floating spheres as the targets
%and a robot arm as the sensor system.

%The sensor state consists of the {x,y,z,w,qx,qy,qz} arm position where the
%angles are constrained to point towards the target. The euler order is
%always XYZ, but the state is stored as a quaternion.

%The goal is to minimise the weighted target covariance at time T while
%ensuring the goal state is met at time T via A* search.

%Currently only for a single target

% Key Dimensions, n = numTargets:
% x 3nx1
% y 3nx1
% z 2nx1

clearvars
close all

%% Setup parameters
x = [0,0,0,0.707,0,0,-0.707]'; %-90deg about X
yTrue = [1.0, 1.0, 4.0]'; %True target position
yHat = yTrue;% + normrnd(0,3,3,1);
T = 30; %Max time steps
goalThresh = 0.001;

x = alignCamera(x, yHat);

%% Build matrices
E = eye(3); %The initial sigma matrix, units in m^2??
%H is the linearised observation matrix wrt X which is then differentiated
%at the point yHat.

%% Run AStar
%Algorithm 1 from Atanasov2019
%From starting state until time = T, for each potential action
timeSteps = 5; %The heuristic lookahead timesteps
currNode = createANode(x,E,1,[],0,0,0);

OPEN = {};
CLOSED = {}; %Nodes must not be removed from this
goalFlag = false;

[OPEN,currIdx] = addListNode(currNode,OPEN);

tic;
while(size(OPEN,2) > 0)
    % Get a random node
    index = randi(size(OPEN,2));
    currNode = getListNode(OPEN,index);
    
    % Remove the currentNode from the openList
    OPEN = removeListNode(index,OPEN);
    
    % Add the currentNode to the closedList
    [CLOSED,parentIdx] = addListNode(currNode,CLOSED);
    disp("Node closed");
    
    %Goal state reached
    if (toc > T)
        disp("Timeout reached");
        [path,covariance] = constructPath(currNode,CLOSED);
        break;
    end
    if(goalFlag)
        break; %Goal reached at child node
    end
    
    % Generate children
    actions = generateActions(x,27);
    for a = 1:size(actions,2)
        act = actions{a}; 
        
        [childX,childE,childT] = transitionState(currNode.x,currNode.E,currNode.t,yHat,act');
        if (checkGoal(childX,yHat,goalThresh))
            disp("Goal reached at child");
            [path,covariance] = constructPath(currNode,CLOSED);
            goalFlag = true;
            break;
        end
        
        childX = alignCamera(childX, yHat);
        
        child = createANode(childX,childE,childT,parentIdx);
        
        %If child is on closed list
        [inList,index] = checkInList(child,CLOSED);
        if(inList)
            continue;
        end
        
        transCost = log(det(child.E));
        child.g = currNode.g + transCost;
        child.V = getV(child.x,yHat);
        
        child.h = 0;
        child.p = child.g + child.h;
        
        %If child is in open list with lower cost, skip it
        [inList,index] = checkInList(child,OPEN);
        contFlag = 0;
        for n = 1:size(index)
            testNode = getListNode(OPEN,index(n));
            if(child.g > testNode.g)
                contFlag = 1;
            end
        end
        if(contFlag == 1)
            continue;
        end
        
        %Add it to the open list
        [OPEN,~] = addListNode(child,OPEN);
        
    end
    
end
disp("A* Complete");

plotPath(yTrue, yHat, path);

figure();
plot(1:size(covariance,2),covariance);
title('Log Det Covariance');

%{
EKF transition equations

What is the goal function?

Will adding a path length component to the node cost break A*?

The heuristic is path dependent (non-markov) so needs to be evaluated
lazily, is this consistent?

What's a good value for E_0?

Test H

%}





