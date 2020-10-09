function [u,v] = getDetection(cameraPose, expState)
%GETDETECTION Takes the current camera pose (TF from the world frame to the 
%camera frame) and experiment state and
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
translationVector = -1*(translationVector);
rotationMatrix = rotationMatrix';

%This takes the rotation / translation of world coordinates relative to the
%image coords. Ie the inverse of the camera pose
projection = worldToImage(expState.cameraParams,rotationMatrix,translationVector,targetPose);
u = projection(1);
v = projection(2);

ImSize = expState.cameraParams.ImageSize;
%Check if it's in frame
if(u >= ImSize(2)) || (v >= ImSize(1)) || (u <= 0) || (v <= 0)
    disp("Target out of frame");
    u = -1;
    v = -1;
    return
end

%Check if it's within the camera min distance range
camPoints = targetPose*rotationMatrix + translationVector;
if((camPoints(3) < expState.minCamDistance) || (camPoints(3) > expState.maxCamDistance))
    disp("Target out of range (too close/far) of camera");
    u = -1;
    v = -1;
    return
end

%Add noise 
u = u + normrnd(0,expState.imageNoise);
v = v + normrnd(0,expState.imageNoise);

%Clamp noise to keep in frame
if(u >= ImSize(2))
    u = ImSize(2)-1;
end
if(v >= ImSize(1)) 
    v = ImSize(1)-1;
end
if(u <= 0)
    u = 1;
end
if(v <= 0)
    v = 1;
end

%Actual detections are discretised
u = round(u);
v = round(v);

end

