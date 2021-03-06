function alignedCamPose = alignCamera(cameraPose,target,expState)
%ALIGNCAMERA Takes a current camera pose and target position. Changes the
%camera rotation to put the target in the middle of frame.

assert(size(cameraPose,1) == 7);
assert(size(cameraPose,2) == 1);

assert(size(target,1) == 3);
assert(size(target,2) == 1);

alignTo = target - cameraPose(1:3);
alignedCamPose = cameraPose;

%If the camera is already at the goal
if(norm(alignTo) < (1e-10))
    disp("Camera already aligned");
    return;
end
    
alignFrom = [0;0;1];
R_in = quat2rotm(cameraPose(4:7)');
alignFrom = R_in*alignFrom; %Rotate a z vector to align to camera axis

quatOut = vrrotvec(alignFrom,alignTo); %From the VR toolbox
R_change = axang2rotm(quatOut);

R = R_in*R_change; %Apply the required rotation to the current camera orientation

alignedCamPose(4:7) = rotm2quat(R);

end

