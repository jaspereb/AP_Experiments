function actions = getActions(node, actionOffsets)
%GETACTIONS Gets the set of actions from the current node going forwards.
%By applying the action offsets to the state. 

startState = node.x(1:3);

for idx = 1:size(actionOffsets, 1)
    actions(idx,:) = startState + actionOffsets(idx,:)';
end

assert(size(actions,1) == 5);
assert(size(actions,2) == 3);
end

