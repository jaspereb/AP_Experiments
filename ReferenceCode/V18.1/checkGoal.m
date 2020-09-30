function goal = checkGoal(childX,yHat,threshold)
%CHECKGOAL Check if the current node is close to yHat

%Threshold is the distance in world units where the goal is considered
%'reached'

goal = false;
x = childX(1:3);
resid = x - yHat;

if(sum(abs(resid)) < threshold)
    goal = true;
end

end

