function [u,v] = getDetection(cameraPose, expState)
%GETDETECTION Takes the current camera pose and experiment state and
%generates a simulated camera detection in pixel coordinates (u,v). If the
%target is not in frame, u = v = -1

%cameraPose is [x,y,z,qw,qx,qy,qz] where [qw,qx,qy,qz] forms a quaternion

assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);
assert(size(expState.targetPose,1) == 3);
assert(size(expState.targetPose,2) == 1);

targetPose = expState.targetPose';
cameraPose = cameraPose';

translationVector = cameraPose(1:3);
rotationMatrix = quat2rotm(cameraPose(4:7));
projection = worldToImage(expState.cameraParams,rotationMatrix,translationVector,targetPose);
u = projection(1);
v = projection(2);

ImSize = expState.cameraParams.ImageSize;
%Check if it's in frame
if(u >= ImSize(2)) || (v >= ImSize(1)) || (u <= 0) || (v <= 0)
    print("Target out of frame");
    u = -1;
    v = -1;
    return
end

%Add noise 

%Clamp noise to keep in frame


end

