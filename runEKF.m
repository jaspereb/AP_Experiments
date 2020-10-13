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
    obs(:,time) = z;
    
    if(u == -1)
        fprintf("Detection out of frame for pose %i, skipping update step \n", idx);
        continue;
    end
    
    if(zHat(1) == -1)
        fprintf("Estimated detection out of frame for pose %i, skipping update step \n",idx);
        continue;
    end
    
    K{time} = P{time}*C{time}'*inv(R + C{time}*P{time}*C{time}');
    x(:,time) = x(:,time) + K{time}*(zHat - z);
    P{time} = P{time} - K{time}*C{time}*P{time};
end

if(expState.printEKFStatus)
    fprintf('Estimated target pose from %s is %f,%f,%f \n', ...
        expState.currExpName,x(1,end),x(2,end),x(3,end));
    fprintf('Estimate error from %s is %f,%f,%f \n', ...
        expState.currExpName,expState.targetPose(1)-x(1,end), ...
        expState.targetPose(2)-x(2,end),expState.targetPose(3)-x(3,end));
end
if(expState.showFigs)
    plotEstimates(x,expState);
end
end