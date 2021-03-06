function cost = getCost(E, expSet)
%GETCOST Calculate the cost function information component as either 
%the log det, or trace.

%Weighted by the W matrix

if(isequal(expSet.costType,'logdet'))
    cost = diag(diag(E));
    cost = expSet.W*cost; %Weighted diagonal covariance
    cost = log(det(cost));
    return;
    
elseif(isequal(expSet.costType,'logtrace'))
    cost = diag(diag(E));
    cost = expSet.W*cost; %Weighted diagonal covariance
    cost = log(trace(cost));
    return;
    
elseif(isequal(expSet.costType,'trace'))
    cost = diag(diag(E));
    cost = expSet.W*cost; %Weighted diagonal covariance
    cost = trace(cost);
    return;
else
    disp("Unknown cost function, the options are logdet or trace");
    cost = -inf;
    return;
end
end

