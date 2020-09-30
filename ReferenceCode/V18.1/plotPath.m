function plotPath(yTrue, yHat, paths, covMat)
%PLOTPATH Takes the sensor state, target state, path, and optionally the target
%covariance. Plots these in 3d.

plotLines = true;

figure(1);
hold off;
plot3(yTrue(1),yTrue(2),yTrue(3),'r*');
hold on;
plot3(yHat(1),yHat(2),yHat(3),'ro');

axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on

if(~iscell(paths))
    for t = 1:size(paths,2)
        x = paths(:,t);
        %Default camera pose is Z=Z, Y=-Y, X=-X so first align this to world axes
        Ro = [-1,0,0;0,-1,0;0,0,1];
        R = quat2rotm(x(4:7)');
        Rc = R*Ro; %Postmultiple because XYZ order and this is 180deg about Z
        
        plotCamera('Location',x(1:3),'Orientation',Rc,'Opacity',0, 'Size', 0.1, 'Label',num2str(t));
        %The plotcamera rotation is about the camera axes, not the world axes!
        
        if(plotLines)
            plot3([x(1),yHat(1)], [x(2),yHat(2)], [x(3),yHat(3)],'k-');
        end
    end
else %plot multiple paths
    assert(size(paths,2) == 3)
    path = paths{1,1};
    for t = 1:size(path,2)
        x = path(:,t);
        %Default camera pose is Z=Z, Y=-Y, X=-X so first align this to world axes
        Ro = [-1,0,0;0,-1,0;0,0,1];
        R = quat2rotm(x(4:7)');
        Rc = R*Ro; %Postmultiple because XYZ order and this is 180deg about Z
        disp(x(1:3)');
        plotCamera('Location',x(1:3),'Orientation',Rc,'Opacity',0, 'Size', 0.1, 'Color', [1,0,0]);
        %The plotcamera rotation is about the camera axes, not the world axes!
        
        if(plotLines)
            plot3([x(1),yHat(1)], [x(2),yHat(2)], [x(3),yHat(3)],'k-');
        end
    end
    path = paths{1,2};
    for t = 1:size(path,2)
        x = path(:,t);
        %Default camera pose is Z=Z, Y=-Y, X=-X so first align this to world axes
        Ro = [-1,0,0;0,-1,0;0,0,1];
        R = quat2rotm(x(4:7)');
        Rc = R*Ro; %Postmultiple because XYZ order and this is 180deg about Z
        
        plotCamera('Location',x(1:3),'Orientation',Rc,'Opacity',0, 'Size', 0.1, 'Color', [0,1,0]);
        %The plotcamera rotation is about the camera axes, not the world axes!
        
        if(plotLines)
            plot3([x(1),yHat(1)], [x(2),yHat(2)], [x(3),yHat(3)],'k-');
        end
    end
    path = paths{1,3};
    for t = 1:size(path,2)
        x = path(:,t);
        %Default camera pose is Z=Z, Y=-Y, X=-X so first align this to world axes
        Ro = [-1,0,0;0,-1,0;0,0,1];
        R = quat2rotm(x(4:7)');
        Rc = R*Ro; %Postmultiple because XYZ order and this is 180deg about Z
        
        plotCamera('Location',x(1:3),'Orientation',Rc,'Opacity',0, 'Size', 0.1, 'Color', [0,0,1]);
        %The plotcamera rotation is about the camera axes, not the world axes!
        
        if(plotLines)
            plot3([x(1),yHat(1)], [x(2),yHat(2)], [x(3),yHat(3)],'k-');
        end
    end
    
end

if(exist('covMat','Var'))
    ellipsoid(yHat(1),yHat(2),yHat(3),covMat(1,1),covMat(2,2),covMat(3,3));
end

axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on

end
