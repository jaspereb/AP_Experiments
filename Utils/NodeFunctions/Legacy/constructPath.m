function [path,cost,diagonals] = constructPath(node,LIST,expSet)
%CONSTRUCTPATH Trace node parents to reconstruct the path (list of states),
%the cost (cost function at each step) and the diagonal covariance
%components.

cost = getCost(node.E,expSet);

path = [];
path(:,1) = node.x(1:7);

diagonals = [];
diagonals(:,1) = diag(node.E);
visibilities = node.visible;

while(~isempty(node.parent))
    node = getListNode(LIST,node.parent);
    path(:,end+1) = node.x(1:7);
    cost(end+1) = getCost(node.E,expSet);
    diagonals(:,end+1) = diag(node.E);
    visibilities(end+1) = node.visible;
end

visibilities = flip(visibilities);
path = flip(path,2);
cost = flip(cost);
diagonals = flip(diagonals,2);

for n = 1:size(path,2)
    if(~visibilities(n))
        fprintf("CAUTION: Non visible node on path at pos %d \n",n);
    else
        fprintf("Added node to path at pos %d \n",n);
    end
    
end

