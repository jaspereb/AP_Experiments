function H = dummyHeuristic(node, yHat)

x1 = node.x(1:3);
x2 = yHat;

%Admissible
% H = sqrt((x1(1)-x2(1))^2 + (x1(2)-x2(2))^2 + (x1(3)-x2(3))^2);

%Non admissible
H = sum(abs(x1-x2));

% H = H - 1*rand(1);
% H = max(H,0);

end

