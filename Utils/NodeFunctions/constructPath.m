function [path,cost,diagonals] = constructPath(node,expState)
%CONSTRUCTPATH Trace node parents to reconstruct the path (list of cameraPoses),
%the cost (Sigma trace at each step) and the diagonal covariance
%components.

cost = trace(node.Sigma);

path = [];
x = node.x;
cameraPose = [x;0;0;0;1];
cameraPose = alignCamera(cameraPose,expState.targetPose,expState);
path(:,1) = cameraPose;

diagonals = [];
diagonals(:,1) = diag(node.Sigma);

while(~isempty(node.parent))
    node = node.parent;
    x = node.x;
    cameraPose = [x;0;0;0;1];
    cameraPose = alignCamera(cameraPose,expState.targetPose,expState);
    path(:,end+1) = cameraPose;
    cost(end+1) = trace(node.Sigma);
    diagonals(:,end+1) = diag(node.Sigma);
end

path = flip(path,2);
cost = flip(cost);
diagonals = flip(diagonals,2);
