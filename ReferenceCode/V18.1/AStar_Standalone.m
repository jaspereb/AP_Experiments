%Standalone version of the A* code for testing. Normally this is called
%from runComparison.m

clearvars
close all

%% Experiment Settings
expSet.actions = 18;
expSet.initialState = [0,0,0,0.707,0,0,-0.707]';
expSet.yTrue = [0.0, 0.0, 4.0]';
expSet.yHat = expSet.yTrue;% + normrnd(0,0.5,3,1);
expSet.Ezero = 0.001*eye(3);
expSet.stepSize = 0.3; %Distance per action step
expSet.goalThresh = expSet.stepSize*2;
expSet.sensorNoiseStd = 5.0; %Std dev of the sensor noise in pixels
expSet.gamma = 1.0; %Tuning factor for explore/exploit
expSet.heuristicTimesteps = 4; %The heuristic lookahead timesteps
expSet.W = [1,0,0;0,1,0;0,0,1];
expSet.costType = 'trace'; %Options are 'logdet','trace'
expSet.constDist = true; %Generate actions of constant distance
expSet.numActions = 27; %Number of action choices at each step
expSet.minSensorDist = 0.2; %Minimum sensor range, below this no info is gained
expSet.maxSensorDist = 5; %Max sensor range, above this no info is gained
expSet.dominatedDistance = 0.5*expSet.stepSize; %Distance threshold to consider a node dominated
expSet.visIntermediatePaths = true; %Show each path as it is expanded

%% Setup parameters
x = expSet.initialState;
yTrue = expSet.yTrue;
yHat = expSet.yHat;
currNode = createANode(x,expSet.Ezero,1,1,[],0,0,0);
currNode.visible = 1;
maxActions = expSet.actions;
E = expSet.Ezero;

OPEN = {};
CLOSED = {}; %Nodes must not be removed from this
goalFlag = false;

[OPEN,currIdx] = addListNode(currNode,OPEN);

%Pre check if goal is actually reachable
minSteps = stepsToReach(expSet.initialState, expSet);
if(minSteps > expSet.actions)
    fprintf("Goal not reachable in the set number of actions, %d actions required\n",minSteps);
    return;
else
    fprintf("Goal reachable in %d actions\n",minSteps);
end

fprintf("Running A* with a lookahead heuristic of %d steps \n", expSet.heuristicTimesteps);
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
        disp("Action timeout reached");
        [path,cost,diagonals] = constructPath(currBestNode,CLOSED,expSet);
        break;
    end
    if(goalFlag)
        break; %Goal reached at child node
    end
    
    %If the current node can't see the target
    if(~currNode.visible)
        disp("Entered non visible node");
    end
    
    %Visualise the current node expansion
    if(expSet.visIntermediatePaths)
        [path,~,~] = constructPath(currNode,CLOSED,expSet);
        plotPath(yTrue, yHat, path);
    end
    
    % Generate children
    actions = generateActions(currNode.x,expSet);
    for a = 1:size(actions,2)
        act = actions{a};
        
        [childX,childE,childT,childVis] = transitionState(currNode.x,currNode.E,currNode.t,yHat,act');
        if (checkGoal(childX,yHat,expSet.goalThresh))
            disp("Goal reached at child");
            [path,cost,diagonals] = constructPath(child,CLOSED,expSet);
            goalFlag = true;
            break;
        end
        
        child = createANode(childX,childE,childT,childVis,parentIdx);
        
        %Ensure range margin is > 0
        minSteps = stepsToReach(child.x, expSet);
        if((minSteps+child.t) > expSet.actions)   
            continue;
        end
        
        %If child is on closed list
        [inList,index] = checkInList(child,CLOSED,expSet);
        if(inList)
            disp("Skipping already closed node");
            continue;
        end

        childCost = getCost(child.E,expSet);
        if(childCost == -inf || childCost == inf)
            disp("Infinite cost error");
            continue;
        end
        
        child.g = currNode.g + childCost;
        child.V = getV(child.x,yHat);
        child.h = evalHeuristic(child, yHat, expSet);
%         child.h = 0;
        child.p = child.g + child.h;
        
        %If child is in open list with lower cost, skip it
        [inList,index] = checkInList(child,OPEN,expSet);
        contFlag = 0;
        for n = 1:size(index)
            testNode = getListNode(OPEN,index(n));
            %We already found another route to that position which is
            %faster and also more informative
            if((child.g > testNode.g))% && (child.t > testNode.t))
                contFlag = 1; %Can ignore t because the cost is cumulative
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
% STILL NOT DOING NODE DOMINANCE PROPERLY


aStarResults.finalE = currBestNode.E;
aStarResults.cost = cost;
aStarResults.path = path;

plotPath(yTrue, yHat, path);

figure(2);
plot(1:size(cost,2),cost, 'k');
title(strcat(expSet.costType,' Covariance'));

figure(3);
hold on
plot(1:size(diagonals,2),diagonals(1,:),'r-*');
plot(1:size(diagonals,2),diagonals(2,:),'g--');
plot(1:size(diagonals,2),diagonals(3,:),'b-');
title('Covariance Components');
legend('X','Y','Z');

tilefigs;

%%Check here if
% -add code to properly terminate search around goal rather than before it
% -add min dist code for D435






