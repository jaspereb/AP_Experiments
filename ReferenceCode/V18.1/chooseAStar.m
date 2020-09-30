%This script simulates an active perception problem from Atanasov2019 where
%there are a series of fixed targets, a sensor model and some system
%control input model. This is specific to floating spheres as the targets
%and a robot arm as the sensor system.

%The sensor state consists of the {x,y,z,w,qx,qy,qz} arm position for a 
%cartesian manipulator. The euler order is
%always XYZ, but the state is stored as a quaternion.

%The goal is to minimise the weighted target covariance at time T while
%ensuring the goal state is met at time T via A* search.

%Currently only for a single target

% Key Dimensions, n = numTargets:
% x 3nx1
% y 3nx1
% z 2nx1

%% Setup parameters
x = expSet.initialState;
yTrue = expSet.yTrue;
yHat = expSet.yHat;
currNode = createANode(x,expSet.Ezero,1,1,[],0,0,0);
maxActions = expSet.actions;
E = expSet.Ezero;
stepSize = expSet.stepSize;
goalThresh = expSet.goalThresh;

timeSteps = 4; %The heuristic lookahead timesteps
fprintf("Running A* with a lookahead heuristic of %d steps \n", timeSteps);

OPEN = {};
CLOSED = {}; %Nodes must not be removed from this
goalFlag = false;

[OPEN,currIdx] = addListNode(currNode,OPEN);

while(size(OPEN,2) > 0)
    % Let the currentNode equal the node with the least f value
    [currNode,index] = getMinP(OPEN,false);
    
    % Remove the currentNode from the openList
    OPEN = removeListNode(index,OPEN);
    
    % Add the currentNode to the closedList
    [CLOSED,parentIdx] = addListNode(currNode,CLOSED);
    disp("Node closed");
    
    %Goal timeout reached
    currBestNode = getMinP(CLOSED,true);
    if (currBestNode.t > maxActions-1)
        [path,cost] = constructPath(currBestNode,CLOSED);
        break;
    end
    if(goalFlag)
        break; %Goal reached at child node
    end
    
    %If the current node can't see the target
    if(~currNode.visible)
        disp("Entered non visible node");
    end
        
    % Generate children
    actions = generateActions(currNode.x,expSet);
    for a = 1:size(actions,2)
        act = actions{a}; 
        
        [childX,childE,childT,childVis] = transitionState(currNode.x,currNode.E,currNode.t,yHat,act');
        if (checkGoal(childX,yHat,goalThresh))
            disp("Goal reached at child");
            currBestNode = getMinP(CLOSED,true);
            [path,cost] = constructPath(currBestNode,CLOSED,expSet);
            goalFlag = true;
            break;
        end
                
        child = createANode(childX,childE,childT,childVis,parentIdx);
        
        %If child is on closed list
        [inList,index] = checkInList(child,CLOSED,expSet);
        if(inList)
            continue;
        end
        
        transCost = log(det(child.E));
        child.g = currNode.g + transCost;
        child.V = getV(child.x,yHat);
        
        child.h = evalHeuristic(child, yHat, timeSteps, stepSize);
        child.p = child.g + child.h;
        
        %If child is in open list with lower cost, skip it
        [inList,index] = checkInList(child,OPEN,expSet);
        contFlag = 0;
        for n = 1:size(index)
            testNode = getListNode(OPEN,index(n));
            %We already found another route to that position which is
            %faster and also more informative
            if((child.g > testNode.g) && (child.t > testNode.t))
                contFlag = 1;
                disp("Skipping dominated node");
            end
        end
        if(contFlag == 1)
            continue;
        end
        
        %Add it to the open list
        [OPEN,~] = addListNode(child,OPEN);
        
    end
    
end

aStarResults.finalE = currBestNode.E;
aStarResults.cost = cost;
aStarResults.path = path;

% plotPath(yTrue, yHat, path);
% 
% figure();
% plot(1:size(covariance,2),covariance);
% title('Log Det Covariance');

%{
EKF transition equations

Will adding a path length component to the node cost break A*?

%}



