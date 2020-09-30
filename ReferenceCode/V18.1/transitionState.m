function [xPrime,EPrime,tPrime,visible] = transitionState(x,E,t,yHat,action)
%TRANSITIONSTATE This function transitions the augmented state. It takes
%the current x,E,t plus the current target estimate and an action, and 
%returns the next timestep for these.

%This is line 6 from Algorithm 1 in Atanasov2019. 

% INPUTS:
% x = the 6 element arm state [x,y,z,r,p,y]
% E = the sigma covariance matrix at time t
% yHat = the current target state estimate, in order to set the camera
% orientation
% action = a vector of the incremental state change for x(1:3)
% t = the current time step

assert(isequal(size(x),[7,1]));
assert(isequal(size(action),[3,1]));
assert(isequal(size(E),[3,3]));
assert(isequal(size(yHat),[3,1]));

% X transition
xPrime(1:3) = x(1:3) + action;
xPrime(4:7) = x(4:7);
xPrime = xPrime';

%System model
A = getA();
W = getW();
V = getV(xPrime,yHat);

% E transition
%Predict
E = A*E*A' + W;

%Update
C = generateH(xPrime,yHat);
R = C*E*C' + V;
K = E*C'*inv(R);
F = eye(size(A)) - K*C;
EPrime = F*E;

% t transition
tPrime = t + 1;

%If a target is visible from that node
if(isequal(zeros(size(C)),C))
    visible = false;
else
    visible = true;
end

end

