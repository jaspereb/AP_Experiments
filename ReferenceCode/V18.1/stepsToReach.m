function minSteps = stepsToReach(x,expSet)
%STEPSTOREACH Runs a greedy action search to find the min number of actions to
%reach the goal. Assumes no obstacles but arbitrary actions model.

arbitraryActions = false; %Use the fast model instead
yHat = expSet.yHat;

%% Runs a greedy action search to find the min number of actions to
% reach the goal. Assumes no obstacles but arbitrary actions model.
if(arbitraryActions)
    minSteps = 0;
    while(~checkGoal(x,yHat,expSet.goalThresh)) %While not at the goal
        minSteps = minSteps + 1;
        actions = generateActions(x,expSet);

        xPrime = [];
        dists = [];
        for n = 1:size(actions,2)
            xPrime(:,n) = x;
            xPrime(1:3,n) = xPrime(1:3,n) + actions{n}';
            dists(n) = sqrt(sum((yHat - xPrime(1:3,n)).^2));
        end
        [~,idx] = min(dists);
        x = xPrime(:,idx);

    end
    minSteps = minSteps + ceil(expSet.goalThresh/expSet.stepSize); %To account for the goal threshold
    
%% Assumes an action set of 0 or 25 deg angles. Ignores the goal threshold setting
else
    assert(expSet.numActions == 27);
    deltas = (yHat-x(1:3));
    deltas = sort(abs(deltas)); %Sort ascending
    
    minDist = deltas(3) + (sqrt(2)-1)*deltas(2) + (sqrt(2)-1)*deltas(1);
    minSteps = ceil(minDist/expSet.stepSize);
    
end

