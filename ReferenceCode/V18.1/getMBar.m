function M_bar = getMBar(child,yHat,timeSteps,expSet)
%GETMBAR Generate the M bar matrix for a given timestep and the minimum
%distance from the reachable set to the goal

x = child.x;
V = child.V;

%2x3 camera matrix
intrinsics = getD435Intrinsics(1);
K = [intrinsics.fx,0,intrinsics.ppx;...
    0,intrinsics.fy,intrinsics.ppy];

KV = K'*(V\K);
lambda_KV = eigs(KV,1);

pSet = getReachableSet(x,timeSteps,expSet.stepSize);

%Filter the remaining for range margin < 0
idx = [];
actsLeft = (expSet.actions-(child.t-1+timeSteps));
for n = 1:size(pSet,2)
    minSteps = stepsToReach(pSet(:,n),expSet);
    if(minSteps > actsLeft)
        idx = [idx;n];
    end
end
pSet(:,idx) = [];

if(size(pSet,2) < 1)
    disp("Error: Heuristic reachable set does not include goal path");
end

%Find the maximum information matrix value over the reachable set (see (32)
%in Atanasov2019)
trans = repmat(yHat,1,size(pSet,2)) - pSet;
norms = (vecnorm(trans,2,1)).^2; %Vector norm of each column
infoVal = -inf*ones(size(pSet,2),1);

%For cartesian orientation only
R = x(4:7);
R = quat2rotm(R');

for n = 1:size(pSet,2)       
    %Check visibility
    piMat = R'*(yHat-pSet(:,n));
    piMat = (1/piMat(3))*piMat;
    z = K*piMat; %This is [u,v]
    if((min(z) < 0) || (z(1) > intrinsics.width) || (z(2) > intrinsics.height))
        infoVal(n) = -inf;
        continue;
    end
    
    denom = ([0,0,1]*R'*(yHat - pSet(:,n)))^4;
    infoVal(n) = norms(n)/denom;
end

pMax = max(infoVal);
if(pMax == -inf)
    M_bar = zeros(3);
else
    M_bar = lambda_KV*eye(3)*pMax;
end

end

