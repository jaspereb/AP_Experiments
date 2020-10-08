function [obs,inFrame] = ObsFunction(state, cameraPose, expState)
%G The observation function, takes the filter state (estimated target pose)
%and the current camera position. Returns the predicted pixel locations for
%where the detection would fall.

%state is the estimated target pose as a column vector
%cameraPose is the camera pose as [x,y,z,qw,qx,qy,qz]'
%expState contains the noise values to add and camera parameters

%If the pixel coordinates fall outside the image frame, they are both
%returned as -1

%'worldToImage' can't be used with calcJac, but the output of ObsFunction
%and getDetection should be the same

%Note: the values of u,v must not be manually set (as an out of frame flag)
%because this breaks complex differentiation 

assert(size(state,1) == 3);
assert(size(state,2) == 1);

assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);

intrinsics = expState.cameraParams.Intrinsics;

cameraPose = cameraPose';
T = cameraPose(1:3);
R = quat2rotm(cameraPose(4:7));

T = -1*T;
R = R';

%Put points from world frame into camera frame
camPoints = state'*R + T;

% u = f*(X/Z) + o_x and v = f*(Y/Z) + o_y
% From http://www.cse.psu.edu/~rtc12/CSE486/lecture13.pdf p14
% Assumes camera axes are aligned with pixel axes (+x ~ +u)
u = intrinsics.FocalLength(1)*(camPoints(1)/camPoints(3)) + intrinsics.PrincipalPoint(1);
v = intrinsics.FocalLength(2)*(camPoints(2)/camPoints(3)) + intrinsics.PrincipalPoint(2);

inFrame = true;
if(min([u,v]) <= 0) || (u >= intrinsics.ImageSize(2)) || (v >= intrinsics.ImageSize(1))
    inFrame = false;
end

obs = [u;v];

end