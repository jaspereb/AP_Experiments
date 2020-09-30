function [listOut,index] = addListNode(node,listIn)
% Add a node to a list

listOut = listIn;
index = size(listIn,2) + 1;
listOut{index} = node;

end

