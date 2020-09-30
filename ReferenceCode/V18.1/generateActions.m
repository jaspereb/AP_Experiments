function [actions] = generateActions(state,expSet,goal)
%GENERATEACTIONS Take the current sensor state, a goal state and generates
%possible actions from this state. expSet.numActions can be 27 or 18,  6 or 5

%If constDist is specified then it will generate actions that are a
%constant traversal distance in the L2 norm (they lie on a sphere).

actions = {};
stepSize = expSet.stepSize;

%Generate a list of the 6 possible cartesian directions, if goal is
%specified then remove the direction that moves away from it most.
if(expSet.numActions==6 || expSet.numActions==5)
    actions{1} = [-stepSize,0,0];
    actions{2} = [stepSize,0,0];
    actions{3} = [0,-stepSize,0];
    actions{4} = [0,stepSize,0];
    actions{5} = [0,0,-stepSize];
    actions{6} = [0,0,stepSize];
    
    if(expSet.expSet.numActions==5)
        assert(exist('goal','Var'));
        offset = goal - state(1:3);
        [~,badDir] = max(abs(offset));
        
        if(offset(badDir) > 0) %Don't move in negative dir
            rmId = (badDir-1)*2 + 1;
        else %Don't move in positive dir
            rmId = (badDir-1)*2 + 2;
        end
        actions(rmId) = [];
    end
    return;
end

%Generate a list of the 27 possible cartesian directions, if goal is
%specified then remove the direction that moves away from it most.
if(expSet.numActions==18)
    assert(exist('goal','Var'));
    offset = goal - state(1:3);
    [~,badDir] = max(abs(offset));
    
    %There is definitely a better way of doing this
    if(offset(badDir) > 0) %Don't move in negative dir
        switch badDir
            case 1 %X
                for xstep = 0:stepSize:stepSize
                    for ystep = -stepSize:stepSize:stepSize
                        for zstep = -stepSize:stepSize:stepSize
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
            case 2 %Y
                for xstep = -stepSize:stepSize:stepSize
                    for ystep = 0:stepSize:stepSize
                        for zstep = -stepSize:stepSize:stepSize
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
            case 3 %Z
                for xstep = -stepSize:stepSize:stepSize
                    for ystep = -stepSize:stepSize:stepSize
                        for zstep = 0:stepSize:stepSize
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
        end
        
    else %Don't move in positive dir
        switch badDir
            case 1 %X
                for xstep = -stepSize:stepSize:0
                    for ystep = -stepSize:stepSize:stepSize
                        for zstep = -stepSize:stepSize:stepSize
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
            case 2 %Y
                for xstep = -stepSize:stepSize:stepSize
                    for ystep = -stepSize:stepSize:0
                        for zstep = -stepSize:stepSize:stepSize
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
            case 3 %Z
                for xstep = -stepSize:stepSize:stepSize
                    for ystep = -stepSize:stepSize:stepSize
                        for zstep = -stepSize:stepSize:0
                            actions{end+1} = [xstep,ystep,zstep];
                        end
                    end
                end
        end
    end
else %No goal specified
    for xstep = -stepSize:stepSize:stepSize
        for ystep = -stepSize:stepSize:stepSize
            for zstep = -stepSize:stepSize:stepSize
                actions{end+1} = [xstep,ystep,zstep];
            end
        end
    end
end

%Remove zero case
rmID = [];
for n = 1:size(actions,2)
    if(isequal(actions{n},[0,0,0]))
        rmID = [rmID;n];
    end
end
actions(rmID) = [];

%If constDist, treat each action as a unit vector and multiply it by the
%step size
if(expSet.constDist)
    for n = 1:size(actions,2)
        actions{n} = normalize(actions{n},'norm',2);
        actions{n} = stepSize.*actions{n};
    end
end


end

