function [z,J] = calcJac(funct, state, cameraPose, expState)
%CALCJAC [nextState, Jacobian] = calcJac(functionHandle, state, cameraPose, expState)
%   This function calculates the state evolution and jacobian of a function

%J = 
% [dz1/dx1 dz1/dx2 ... dz1/dxn
%  dz2/dx1 ...         dz2/dxn
%  ...                  ...
%  dzn/dx1 ...         dzn/dxn]

% So the Jacobian has shape [elements(observations), elements(states)]

%This code is specific to the VS EKF task

[z,inFrame] = funct(state, cameraPose, expState);
if(~inFrame)
    z = [-1,-1];
    disp("CAUTION: Pixel outside frame when finding Jacobian");
end

cols = numel(state);
rows = numel(z);

J = zeros(rows,cols);
h = cols*eps;

for k = 1:cols
    xi = state;
    xi(k) = xi(k) + h*i; %x(i) with a complex part
    [result,~] = funct(xi, cameraPose, expState);
    J(:,k) = imag(result)/h; %See differentiation by complex parts
    
end
end

