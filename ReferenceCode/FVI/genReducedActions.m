function [actions] = genReducedActions(state,goal)
%GENERATEACTIONS Take the current sensor state, a goal state and generates
%possible actions from this state
stepSize = 0.1;
actions = {};

%Generate a list of the 6 possible cartesian directions, if goal is
%specified then remove the direction that moves away from it most.

actions{1} = [-stepSize,0,0]';
actions{2} = [stepSize,0,0]';
actions{3} = [0,-stepSize,0]';
actions{4} = [0,stepSize,0]';
actions{5} = [0,0,-stepSize]';
actions{6} = [0,0,stepSize]';
    
if(exist('goal','Var'))
    offset = goal - state(1:3);
    [~,badDir] = max(abs(offset));
    
    if(offset(badDir) > 0) %Don't move in negative dir
        rmId = (badDir-1)*2 + 1;
    else %Don't move in positive dir
        rmId = (badDir-1)*2 + 2;
    end   
    actions(rmId) = [];
end
end

