function camPoint = toCameraAxes(cameraPose, point)
%TOCAMERAAXES Transforms a point from world to camera axes

assert(size(point,1) == 3);
assert(size(point,2) == 1);
assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);

cameraPose = cameraPose';
T = cameraPose(1:3);
R = quat2rotm(cameraPose(4:7));

%To make the world axes coincident with camera axes move and rotate them.
%Because T is expressed in world axes it needs to be applied before the
%rotation.
camPoint = (point'-T)*R;
end

