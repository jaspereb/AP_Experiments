%Experiment 2 generates an estimate from 1000 runs of the best path found
%using offline FVI

load('StartState.mat');
[runState,x,C] = getRandTarget(expState);

% Initialise the starting node
openList{1} = createANode(expState.initialPose(1:3),P{1},[]);
actionOffsets = getActionOffsets(runState);

% Run FVI 
for step = 2:expState.numPoses
    fprintf("Building FVI tree at step %d of %d with max %d nodes at this level \n", ... 
        step, expState.numPoses,size(openList{step-1},2)*size(actions));
    
    % For each open node
    for currentIdx = 1:size(openList{step-1},2)
        parentNode = openList{step-1}(currentIdx);
        actions = getActions(parentNode, actionOffsets);
        
        % For each action in the action space
        for actionIdx = 1:size(actions,1)
            action = actions(actionIdx,:);
            childNode = transitionState(parentNode, action);
            
            %Check if node is dominated and add it
            openList = addNode(openList, step, childNode);
            openList{step}(end+1) = childNode;
            
        end
        
    end
    
    
    
end








% TO RUN FVI



% Determine the best estimated path
FVIPath = getFVIPath(cameraPose,expState);

% Run this 1000 times
expState.currExpName = 'FVI Offline Path';
if(expState.showFigs)
    fig_AP = figure(3);
    plotState(APPoses, expState, fig_AP);
end
[x_AP,P_AP] = runEKF(FVIPath,expState,functH,x,Q,R,A,K,C,P);
