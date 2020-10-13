function list = addNode(list,level,node)
%ADDNODE Check if a node is dominated/dominates in a list and add it
%otherwise. List is one level of the FVI tree. 

ep = 1e-10; %Numerical rounding factor

% Check if its in the list


WORKING HERE, NEED TO MAKE IT A CELL ARRAY OF STRUCTS. 
% Add it
list{level}(end+1) = node;
list{2}{end+1} = node
list{2} = []
end

