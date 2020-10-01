function [x,P,obs] = runEKF(poses,expState,functH,x,Q,R,A,K,C,P)
%RUNEKF takes the camera poses list, experiment state, obeservation
%function handle and EKF matrices. Runs an EKF filter over the simulated
%camera obs and returns the state estimate over time along with state covar
%matrix P

for idx = 1:size(poses,2)
    time = idx + 1; %Time starts from 2
    x(:,time) = A*x(:,time-1);
    
    cameraPose = poses(:,idx);
    if(isequal(cameraPose,expState.grabPose))
        print("Reached target");
        break;
    end
    
    xPrior(:,time) = x(:,time);
    P{time} = A*P{time-1}*A' + Q;
    
    [zHat,C{time}] = calcJac(functH, x(:,time), cameraPose, expState);
    [u,v] = getDetection(cameraPose, expState);
    z = [u;v];
    
    K{time} = P{time}*C{time}'*inv(R + C{time}*P{time}*C{time}');
    x(:,time) = x(:,time) + K{time}*(zHat - z);
    obs(:,time) = z;
    P{time} = P{time} - K{time}*C{time}*P{time};
end

fprintf('Estimated target pose from straight trajectory for experiment %s is %f,%f,%f \n', ...
    expState.currExpName,x(1,end),x(2,end),x(3,end));
plotEstimates(x,expState);

end