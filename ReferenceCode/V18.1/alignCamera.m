function xOut = alignCamera(xIn, yHat)
%ALIGNCAMERA Alter the state vector so that the camera aligns to the
%current target estimate

alignFrom = [0;0;1];

if(isequal(yHat, xIn(1:3)))
    quatOut = [1,0,0,0];
else
    alignTo = yHat - xIn(1:3);
    xOut = xIn;
    quatOut = vrrotvec(alignFrom,alignTo); %From the VR toolbox
    quatOut = axang2quat(quatOut);
end
xOut(4:7) = quatOut;


% % %Now try and rotate around the z axis to make the y axis point up
% % %(along world Z)
% % %Angle from Z_world to Y'
% % vecNew = rotM*[0;1;0];
% % zAng = -1*atan2(norm(cross([0,0,1],vecNew)),dot([0,0,1],vecNew));
% % rotM2 = eul2rotm([0,0,zAng],'XYZ');
% % % rotM = rotM*rotM2;
% % rotM = rotM2*rotM;


end

