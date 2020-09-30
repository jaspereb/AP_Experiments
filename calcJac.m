function [z,J] = calcJac(funct,x, camPos, camRot, intrinsics)
%CALCJAC [nextState, Jacobian] = calcJac(functionHandle, state, camPos, camRot, intrinsics)
%   This function calculates the state evolution and jacobian of a function

%J = 
% [dz1/dx1 dz1/dx2 ... dz1/dxn
%  dz2/dx1 ...         dz2/dxn
%  ...                  ...
%  dzn/dx1 ...         dzn/dxn]

% So the Jacobian has shape [elements(observations), elements(states)]

%This code is specific to the visual servoing task


z = funct(x, camPos, camRot, intrinsics);
cols = numel(x);
rows = numel(z);

J = zeros(rows,cols);
h = cols*eps;

for k = 1:cols
    xi = x;
    xi(k) = xi(k) + h*i; %x(i) with a complex part
    J(:,k) = imag(funct(xi, camPos, camRot, intrinsics))/h; %See differentiation by complex parts
    
end
end

