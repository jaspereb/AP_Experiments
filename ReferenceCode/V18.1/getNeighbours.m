function [Neighboors, offSets] = getNeighbours(state, Connecting_Distance, MAP)
%GETNEIGHBOURS Return a list of neighbouring nodes for A* as relative
%offsets

useSimpleNeighbours = false;

if(useSimpleNeighbours)
    %Simple neighbours
    Neighboors = [];
    actions = [-Connecting_Distance,0,0];
    Neighboors = [Neighboors; actions];
    actions = [Connecting_Distance,0,0];
    Neighboors = [Neighboors; actions];
    actions = [0,-Connecting_Distance,0];
    Neighboors = [Neighboors; actions];
    actions = [0,Connecting_Distance,0];
    Neighboors = [Neighboors; actions];
    actions = [0,0,-Connecting_Distance];
    Neighboors = [Neighboors; actions];
    actions = [0,0,Connecting_Distance];
    Neighboors = [Neighboors; actions];
else
    %Fully connected neighbours
    Neighboors = [];
    for i=-Connecting_Distance:1:Connecting_Distance
        for j=-Connecting_Distance:1:Connecting_Distance
            for k=-Connecting_Distance:1:Connecting_Distance
                Neighboors(end+1,:) = [i,j,k];
            end
        end
    end
    idx = find(ismember(Neighboors,[0,0,0],'rows'));
    Neighboors(idx,:) = [];
end
offSets = Neighboors;
Neighboors = Neighboors + repmat(state,size(Neighboors,1),1);

rmIdx = [];
%Check each position is in map and not obstacle
for n = 1:size(Neighboors,1)
    pos = Neighboors(n,:);
    
    if(pos(1)<1 || pos(2)<1 || pos(3)<1)
        rmIdx(end+1) = n;
    elseif(pos(1)>size(MAP,1) || pos(2)>size(MAP,2) || pos(3)>size(MAP,3))
        rmIdx(end+1) = n;
    elseif(MAP(pos(1),pos(2),pos(3)) == 1)
        rmIdx(end+1) = n;
    end
end

Neighboors(rmIdx,:) = [];
offSets(rmIdx,:) = [];

