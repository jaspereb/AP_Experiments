function [inList,index] = checkInList(node,list,expSet)
%CHECKINLIST Checks if a given node is already in a list according 
% to the position of that node in 3D space. Returns the index
% of all the list instances for that node.

inList = false;
index = [];
dist = expSet.dominatedDistance;

positions = [];

if(size(list,2) < 1)
    inList = false;
    return;
end

for idx = 1:size(list,2)
    listNode = list{idx};
    positions(1:3,idx) = listNode.x(1:3);
%     
%     if(isequal(node.x(1:3), listNode.x(1:3)))
%         inList = true;
%         index(end+1) = idx;
%     end
    
end

dists = positions - node.x(1:3);
dists = sqrt(sum(dists.^2));

idx = find(dists < dist);

if(isempty(idx))
    inList = false;
else
    inList = true;
end

end

