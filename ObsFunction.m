function G_of_x = ObsFunction(x, camPos, camRot, intrinsics)
%G The observation function, takes a stacked state vector X and returns the
%observations for each point in the stacked state. 

%For the VS task, the state is the [x,y,z,x,y...]T vector of the target
%positions. The observations are the vector [u,v,u,...]T of the pixel
%locations

%Construct the camera matrix
camMat = [intrinsics.fx, 0, intrinsics.ppx; ...
    0, intrinsics.fy, intrinsics.ppy;...
    0,0,1];

%To put points into camera FoR
R = camRot;
T = camPos;
    
G_of_x = [];
for obj = 1:size(x,1)/3
    idx = ((obj-1)*3 +1):((obj-1)*3 + 3);
    P_world = x(idx);

    position = T + R*P_world;
    
    Xc = position(1);
    Yc = position(2);
    Zc = position(3);
    
    projection = camMat*[Xc/Zc;Yc/Zc;1];
    G_of_x = [G_of_x;projection(1:2)];
    
end

if(min(G_of_x) < 0)
    disp("CAUTION: NEGATIVE PIXEL VALUE FOUND");
%     disp(noVar);
end

end