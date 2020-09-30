function [H,z] = generateH(x,y)
%GENERATEH Take the current sensor state (6x1) and a fixed target state (3x1), 
% generate the linearisation of the observation function around the current 
% sensor state. From Atanasov (schlotfeldt) 2019, p5.

%Check input format
assert(isequal(size(x), [7,1]));
assert(isequal(size(y), [3,1]));

%2x3 camera matrix
intrinsics = getD435Intrinsics(1);
K = [intrinsics.fx,0,intrinsics.ppx;...
    0,intrinsics.fy,intrinsics.ppy];

%Camera position and rotation. RPY must be encoded according to the
%sequence below (XYZ).
p = x(1:3);
R = x(4:7);
R = quat2rotm(R');

%Expected observation
piMat = piFun(R'*(y-p));
z = K*piMat;

%Check visibility, if target non-visible then C=0
if((min(z) < 0) || (z(1) > intrinsics.height) || (z(2) > intrinsics.width))
    H = zeros(2,3);
    z = [nan,nan];
    return;
end

%For the camera sensor, Grad(y) = C(x) = K*pi(R'*(y-p))*R'
piMat = piPrimeFun(R'*(y-p));
H = K*piMat*R';

%Pi function
    function piMat = piPrimeFun(piInput)
        assert(isequal(size(piInput), [3,1]));
        
        piMat = (1/(piInput(3)^2));
        piMat = [piInput(3),0,-piInput(1);...
                0,piInput(3),-piInput(2);...
                0,0,0]*piMat;
    end
    
    function piMat = piFun(piInput)
        assert(isequal(size(piInput), [3,1]));      
        piMat = (1/piInput(3))*piInput;
    end
end


