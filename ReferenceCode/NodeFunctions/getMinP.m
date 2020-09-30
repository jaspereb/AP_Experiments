function [node,index] = getMinP(list, getG)
%GETMINP Get the minimum p (estimated path cost) for a list. Returns the
%list node with min cost.

minCost = inf;
index = [];

for idx = 1:size(list,2)
    listNode = list{idx};
    if(getG)
        if(listNode.g < minCost)
            minCost = listNode.g;
            index = idx;
        end
    else
        if(listNode.p < minCost)
            minCost = listNode.p;
            index = idx;
        end
    end
end

node = list{index};

end

