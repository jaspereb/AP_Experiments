function plotState(x, yTrue, yHat, covMat)
%PLOTSTATE Takes the sensor state, target state and optionally the target
%covariance. Plots these in 3d.

%Default camera pose is Z=Z, Y=-Y, X=-X so first align this to world axes
Ro = [-1,0,0;0,-1,0,;0,0,1];
R = eul2rotm(x(4:6)', 'XYZ');
Rc = R*Ro; %Postmultiple because XYZ order and this is 180deg about Z

figure(1);
hold off;
plot3(yTrue(1),yTrue(2),yTrue(3),'r*');
hold on;
plot3(yHat(1),yHat(2),yHat(3),'ro');
cam = plotCamera('Location',x(1:3),'Orientation',Rc,'Opacity',0, 'Size', 0.1);
%The plotcamera rotation is about the camera axes, not the world axes!

axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on

if(exist('covMat','Var'))
    ellipsoid(yHat(1),yHat(2),yHat(3),covMat(1,1),covMat(2,2),covMat(3,3));
end

end
