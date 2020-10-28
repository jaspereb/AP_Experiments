function list = addNode(list,level,node)
%ADDNODE Check if a node is dominated/dominates in a list and add it
%otherwise. List is one level of the FVI tree.

%Level is the relevant FVI tree level

ep = 1e-10; %Numerical rounding factor

%If that level is empty
if(size(list,2) < level)
    list{level}(1) = node;
    return;
end

% Check if its in the list
nodesOnLevel = list{level};
for idx = 1:size(nodesOnLevel,2)
    testNode = nodesOnLevel(idx);
    if(sum(abs(node.x - testNode.x)) < ep)
        
        dominating = node.Cost <= testNode.Cost;
        
        if(dominating)  %Replace the dominated node
            list{level}(idx) = node;
%             disp("Replacing dominated node");
            return
        else %Discard the current node which is dominated
            return
        end
    end
end

% It was not found in the list
list{level}(end+1) = node;

end

