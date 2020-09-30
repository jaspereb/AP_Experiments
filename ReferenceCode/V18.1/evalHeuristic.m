function h_cost = evalHeuristic(child,yHat,expSet)
%EVALHEURISTIC Evaluate the heuristic value for a given node position.
% Based on the camera model heuristic from Atanasov2019

%Check input format
assert(isequal(size(child.x), [7,1]));
assert(isequal(size(yHat), [3,1]));
assert(isequal(size(child.V), [2,2]));

A = getA();
W = getW();

E_k = child.E;
h_cost = 0;

lookAheadSteps = min(expSet.heuristicTimesteps,(expSet.actions-child.t));
for k = 1:lookAheadSteps
    %Forward predict E
    E_k = (A*E_k*A' + W);
    
    %Find the maximum information matrix for k future steps
    M_k = getMBar(child,yHat,k,expSet);
    
    %Update E based on the max info matrix
    E_k = inv(inv(E_k) + M_k);%This is the smallest possible E for k timesteps
    
    %Caclulate the cumulative cost after k timesteps
    h_cost = h_cost + getCost(E_k,expSet);
end

if((child.t + lookAheadSteps) < expSet.actions)
    %If we haven't searched all the way to the end of the path
    %Approximate the cost of the rest of the steps by the min found so far
    %Kind of like a receeding horizon thing
    lastCost = (expSet.actions-child.t-lookAheadSteps)*getCost(E_k,expSet); %Assumes E_k is monotonously decreasing
    h_cost = h_cost + lastCost;
end

end

