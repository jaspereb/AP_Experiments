function cost = estimatorCostFn(measurements, path, K, yHat)
%estimatorCostFn This function implements a pixel space cost function which
%calculates the state which minimises overall pixel error for all measurements.
% This might be equivalent to FIE?

% measurements is the [u_t;v_t] matrix of pixel locations over time

% camRot and camPos are the camera rotation matrix and 3D position time
%aligned to the measurments

% state is the estimated [x_n;y_n;z_n] for 1 to n targets

% K is the 2x3 camera matrix

% Arg Min (p_measured - p_projected)'*(W^-1)*(p_measured - p_projected);
% Where W is a variance weighting matrix, which is identity for now


% ============================= THE CODE =============================

%Currently only for 1 object
assert(isequal(size(yHat),[3,1]));

for t = 1:size(path,2)
    x = path(:,t);
    R = x(4:7);
    R = quat2rotm(R');
    piMat = R'*(yHat-x(1:3));
    piMat = (1/piMat(3))*piMat;
    
    p_projected(t,:) = K*piMat;
    
end
p_measured = measurements';
p_measured = p_measured(:);
p_projected = p_projected(:);
%These vectors are now [u1;u2;u3;...v1;v2;v3...];

%Variance weighting matrix
W = eye(size(p_projected,1));

%Final cost function
cost = (p_measured - p_projected)'*(W^-1)*(p_measured - p_projected);

end

