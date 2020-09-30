function reachableP = getReachableSet(x,t,stepSize)
%GETREACHABLESET Get all the [x,y,z] positions reachable by the arm given t
%timesteps. This should construct a tree using the generateActions
%function, but for simplicity it currently builds a cube in state space.

a = (stepSize*t);
reachableP = [];

for xval = x(1)-a:stepSize:x(1)+a
    for yval = x(2)-a:stepSize:x(2)+a
        for zval = x(3)-a:stepSize:x(3)+a
            reachableP(:,end+1) = [xval;yval;zval]';
        end
    end
end