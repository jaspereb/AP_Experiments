function estState = estInitialState(u,v,depth,cameraPose,expState)
% Estimate the initial target state based on one observation and a depth
% value, if this is unkwnown the trellisDist in expState can be used

%u,v are pixel values
%cameraPose is the camera pose as [x,y,z,qw,qx,qy,qz]'
%expState must have cameraParams, trellisDist

assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);
assert(u > 0);
assert(v > 0);
assert(u < expState.cameraParams.ImageSize(2));
assert(v < expState.cameraParams.ImageSize(1));

intrinsics = expState.cameraParams.Intrinsics;

z = depth;
x = (u - intrinsics.PrincipalPoint(1))/intrinsics.FocalLength(1);
x = x*z;
y = (v - intrinsics.PrincipalPoint(2))/intrinsics.FocalLength(2);
y = y*z;

%Transform the camera points to world frame
estState = fromCameraAxes(cameraPose,[x;y;z]);

end