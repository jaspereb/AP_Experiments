%Experiment 2 generates an estimate from 1000 runs of the best path found
%using offline FVI

%NOTE: within nodes, x refers to the camera pose and y to the target pose.
%Within the EKF x referes to target pose

clearvars  
load('StartState.mat');
expState.currExpName = 'FVI_Offline';

for run = 1:expState.numRuns
    [runState,x,C] = getRandTarget(expState);
    
    % Initialise the starting node
    openList{1} = createANode(runState.initialPose(1:3),x(:,1),runState.P{1},[]);
    actionOffsets = getActionOffsets(runState);
   
    % Run FVI 
    for step = 2:runState.numPoses
        fprintf("Building FVI tree at step %d of %d with max %d nodes at this level \n", ... 
            step, runState.numPoses,size(openList{step-1},2)*5);

        % For each open node
        for currentIdx = 1:size(openList{step-1},2)
            parentNode = openList{step-1}(currentIdx);
            actions = getActions(parentNode, actionOffsets);

            % For each action in the action space
            for actionIdx = 1:size(actions,1)
                action = actions(actionIdx,:);
                childNode = transitionState(parentNode, action, runState);

                %Check if node is dominated and add it
                openList = addNode(openList, step, childNode);

            end
        end
    end
    
    %Find the best path
    traces = [];
    for idx = 1:size(openList{end},2)
       Sigma = openList{end}(idx).Sigma; 
       traces(idx) = trace(Sigma);
    end
    
    [~,idx] = min(traces);
    [path,cost,diagonals,visibilities] = constructPath(openList{end}(idx),runState);
    
    %Run the EKF
    [x_FVI_offline,P_FVI_offline,z_FVI_offline] = runEKF(path,runState,functH,x,K,C);
    FVIofflineResults.x{run} = x_FVI_offline;
    FVIofflineResults.P{run} = P_FVI_offline;
    FVIofflineResults.z{run} = z_FVI_offline;
    FVIofflineResults.targetPose = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end

FVIofflineResults = calculateResults(FVIofflineResults,expState);

if(runState.showFigs)
    fig_fvi = figure();
    plotFVIPath(path, visibilities, openList{end}(idx).y, runState, fig_fvi);
    disp("Displaying example FVI path");
end
