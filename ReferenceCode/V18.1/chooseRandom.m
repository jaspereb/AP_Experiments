%% Random Actions
x = expSet.initialState;
yTrue = expSet.yTrue;
yHat = expSet.yHat;
currNode = createANode(x,expSet.Ezero,1,1,[],0,0,0);
CLOSED = {};
[CLOSED,parentIdx] = addListNode(currNode,CLOSED);

for n = 1:expSet.actions-1
    %Generate a single path choosing random actions
    actions = generateActions(currNode.x,expSet);
    act = actions{randi(size(actions,2))}; 
    [childX,childE,childT,childVis] = transitionState(currNode.x,currNode.E,currNode.t,yHat,act');
    child = createANode(childX,childE,childT,childVis,parentIdx);
    [CLOSED,parentIdx] = addListNode(child,CLOSED);
    currNode = child;
end
[path,cost] = constructPath(child,CLOSED,expSet);
randResults.finalE = child.E;
randResults.cost = cost;
randResults.path = path;