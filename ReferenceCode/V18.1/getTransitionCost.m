function cost = getTransitionCost(node1, node2)
%GETTRANSITIONCOST Calcualte transition cost between two nodes

x1 = node1.x(1:3);
x2 = node2.x(1:3);

cost = sqrt((x1(1)-x2(1))^2 + (x1(2)-x2(2))^2 + (x1(3)-x2(3))^2);

end

