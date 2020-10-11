function worldPoint = fromCameraAxes(cameraPose, point)
%FROMCAMERAAXES Transforms a point from world to camera axes

assert(size(point,1) == 3);
assert(size(point,2) == 1);
assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);

cameraPose = cameraPose';
T = cameraPose(1:3);
R = quat2rotm(cameraPose(4:7));

% T = -1*T;
R = R';

%To make the camera axes coincident with world axes move and rotate them.
%Because T is expressed in world axes it needs to be applied after the
%rotation has aligned them to the world.
worldPoint = point'*R + T;
end

