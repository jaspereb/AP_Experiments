function plotEstimates(x,expState,separateGraphs)
% Plot a series of estimated positions relative to the actual target position

assert(size(x,1) == 3);

if(~exist('separateGraphs'))
    separateGraphs = false;
end

if(separateGraphs)
    a = ones(size(x,2),1)*expState.targetPose(1);
    b = ones(size(x,2),1)*expState.targetPose(2);
    c = ones(size(x,2),1)*expState.targetPose(3);

    figure();
    hold on
    plot(1:size(x,2),x(1,:),'-r'); % x
    plot(1:size(x,2),a,'--r'); % x
    ylabel('X Estimate');
    xlabel('Filter Timestep');
    title(expState.currExpName);

    
    figure();
    hold on
    plot(1:size(x,2),x(2,:),'-g'); % y
    plot(1:size(x,2),b,'--g'); % y
    ylabel('Y Estimate');
    xlabel('Filter Timestep');
    title(expState.currExpName);

    
    figure();
    hold on
    plot(1:size(x,2),x(3,:),'-b'); % z
    plot(1:size(x,2),c,'--b'); % z
    ylabel('Z Estimate');
    xlabel('Filter Timestep');
    title(expState.currExpName);

else
    a = ones(size(x,2),1)*expState.targetPose(1);
    b = ones(size(x,2),1)*expState.targetPose(2);
    c = ones(size(x,2),1)*expState.targetPose(3);

    figure();
    hold on
    plot(1:size(x,2),x(1,:),'-r'); % x
    plot(1:size(x,2),a,'--r'); % x
    plot(1:size(x,2),x(2,:),'-g'); % y
    plot(1:size(x,2),b,'--g'); % y
    plot(1:size(x,2),x(3,:),'-b'); % z
    plot(1:size(x,2),c,'--b'); % z
    title(expState.currExpName);
    xlabel('Filter Timestep');
    xlabel('State Estimate (m)');
    legend('X Estimate','X True','Y Estimate','Y True','Z Estimate','Z True');
end

end