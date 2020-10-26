%Experiment 3 generates an estimate from 1000 runs of the best path found
%using online FVI

%NOTE: within nodes, x refers to the camera pose and y to the target pose.
%Within the EKF x referes to target pose

clearvars  
close all

load('StartState.mat');
expState.currExpName = 'FVI Online';

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
    
    %Find the best path which ends at the target, for grasping
    %Because the domination rule gets applied, only one path per physical
    %end node exists
    for n = 1:size(openList{end},2)
       lastPoses(:,n) = openList{end}(n).x;
    end
    lastDists = lastPoses - x(:,1);
    lastDists = sqrt(sum(lastDists.^2,1));
    [~,idx] = min(lastDists);
    [graspPath,graspCost,graspDiagonals,graspVisibilities] = constructPath(openList{end}(idx),runState);
    
    %Find the best path which ends anywhere, for viewing
    traces = [];
    for idx = 1:size(openList{end},2)
       Sigma = openList{end}(idx).Sigma; 
       traces(idx) = trace(Sigma);
    end
    [~,idx] = min(traces);
    [viewPath,cost,diagonals,visibilities] = constructPath(openList{end}(idx),runState);
    
    %Run the EKF
    [x_FVI_grasp_online,P_FVI_grasp_online,z_FVI_grasp_online] = runEKF(graspPath,runState,functH,x,K,C);
    FVIonlineGraspResults.x{run} = x_FVI_grasp_online;
    FVIonlineGraspResults.P{run} = P_FVI_grasp_online;
    FVIonlineGraspResults.z{run} = z_FVI_grasp_online;
    FVIonlineGraspResults.targetPose{run} = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
    
    %Run the EKF
    [x_FVI_view_online,P_FVI_view_online,z_FVI_view_online] = runEKF(viewPath,runState,functH,x,K,C);
    FVIonlineViewResults.x{run} = x_FVI_view_online;
    FVIonlineViewResults.P{run} = P_FVI_view_online;
    FVIonlineViewResults.z{run} = z_FVI_view_online;
    FVIonlineViewResults.targetPose{run} = runState.targetPose;
    if(mod(run,50) == 0)
        fprintf('\t \t \t \t \t \t \t Executing run %d of %d \n',run, expState.numRuns);
    end
end

disp("Calculating results for end pose constrained path");
expState.currExpName = 'FVI Online Grasp Path';
FVIonlineGraspResults = calculateResults(FVIonlineGraspResults,expState);
disp("Calculating results for non-constrained path");
expState.currExpName = 'FVI Online View Path';
FVIonlineViewResults = calculateResults(FVIonlineViewResults,expState);

if(runState.showFigs)
    fig_fvi_grasp = figure();
    expState.currExpName = 'FVI Online Grasp Path';
    plotFVIPath(graspPath, visibilities, openList{end}(idx).y, runState, fig_fvi_grasp);
    disp("Displaying example FVI grasping path");
    fig_fvi_view = figure();
    expState.currExpName = 'FVI Online View Path';
    plotFVIPath(viewPath, visibilities, openList{end}(idx).y, runState, fig_fvi_view);
    disp("Displaying example FVI viewing path");
end

clearvars fig_fvi_grasp fig_fvi_view
save('Exp3Results.mat');

clearvars
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
save('Exp3Figures.mat');